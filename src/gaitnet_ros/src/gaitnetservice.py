from __future__ import annotations

import gaitnet
import gaitnet.constants as const
# we're not going to talk about this naming...
from gaitnet.gaitnet.gaitnet import GaitnetActor
from gaitnet.gaitnet.components.footstep_candidate_sampler import FootstepCandidateSampler
from gaitnet.simulation.cfg.footstep_scanner_constants import idx_to_xy, xy_to_idx
from gaitnet.util.pga import generate_footstep_action
from pathlib import Path
import torch
import rospy
from gaitnet_interface.srv import (
    GaitNetInterfaceRequest as Req,
    GaitNetInterfaceResponse as Res,
    GaitNetInterface,
)
from gaitnet_interface.msg import FootstepAction, StepCSpaceLeg
import re
from types import SimpleNamespace
from time import perf_counter


args_cli = SimpleNamespace()
args_cli.num_envs = 1  # since we're in a ROS service, we only need one env


leg_name_map: dict[int, str] = {
    FootstepAction.LEG_FL: "front left",
    FootstepAction.LEG_FR: "front right",
    FootstepAction.LEG_RL: "rear left",
    FootstepAction.LEG_RR: "rear right",
}
def pretty_result_str(res: Res) -> str:
    if res.action.noop:
        return "no action"
    
    return leg_name_map.get(res.action.leg, "unknown leg") + f" step to {res.action.hip_offset_xy}"



class GaitNetService:
    def __init__(self, device: torch.device, model_path: Path) -> None:
        self.device = device
        self.model_path = model_path
        self.model: torch.nn.Module = self.load_model(model_path, device)
        rospy.loginfo(
            f"GaitNetService initialized with device {self.device} and model path {self.model_path}"
        )

    @staticmethod
    def load_model(model_path: Path, device: torch.device) -> torch.nn.Module:
        rospy.loginfo(f"Loading model from {model_path} on device {device}")
        model = GaitnetActor(
            shared_state_dim=const.gait_net.robot_state_dim,
            shared_layer_sizes=[128, 128, 128],
            unique_state_dim=const.gait_net.footstep_option_dim,
            unique_layer_sizes=[64, 64],
            trunk_layer_sizes=[128, 128, 128],
        )
        agent = model
        checkpoint = torch.load(model_path, map_location=device, weights_only=True)
        state_dict = checkpoint["model_state_dict"]
        state_dict = {
            re.sub(r"^actor\.", "", k): v
            for k, v in state_dict.items()
            if k.startswith("actor.")
        }
        agent.load_state_dict(state_dict)
        agent.to(device)
        return agent
        
    def _interpret_terrain_data(self, leg_data: StepCSpaceLeg) -> list:
        data = []

        if leg_data.legCSpace is None:
            raise ValueError("legCSpace is None")
        
        # TODO: verify row vs column major order
        for row in leg_data.legCSpace:
            data.extend([1.0 if val else 0.0 for val in row.rowCSpace])
        
        return data
    
    def interpret_request(self, req: Req) -> torch.Tensor:
        # nominal isaaclab ordering: FL FR RL RR
        data: list[float] = []

        # foot positions
        data.extend(req.state.footPositions_xy_b.FL_xy_b)
        data.extend(req.state.footPositions_xy_b.FR_xy_b)
        data.extend(req.state.footPositions_xy_b.RL_xy_b)
        data.extend(req.state.footPositions_xy_b.RR_xy_b)

        # base position
        data.append(req.state.comHeight_z_w)

        # base linear velocity
        data.extend(req.state.linearVelocity_xyz_b)

        # base angular velocity
        data.extend(req.state.angularVelocity_xyz_b)

        # control input
        data.extend(req.control.v_xy_b)
        data.append(req.control.yaw_b)

        # contact state
        data.extend([float(i) for i in [
            req.state.contactState.FL,
            req.state.contactState.FR,
            req.state.contactState.RL,
            req.state.contactState.RR,
        ]])

        # projected gravity
        data.extend(req.state.projectedGravity_xyz_b)

        # terrain data (for whatever reason I didn't make this follow isaaclab ordering)
        ## FR foot
        data.extend(self._interpret_terrain_data(req.stepCSpace.FR))

        ## FL foot
        data.extend(self._interpret_terrain_data(req.stepCSpace.FL))

        ## RR foot
        data.extend(self._interpret_terrain_data(req.stepCSpace.RR))

        ## RL foot
        data.extend(self._interpret_terrain_data(req.stepCSpace.RL))

        return torch.tensor(data, dtype=torch.float32, device=self.device).unsqueeze(0)
    
    def synthesize_footstep_action(self, terrain_obs: torch.Tensor) -> Res:
        best_actions = torch.full(
            (args_cli.num_envs, 4), float("nan"), device=device
        )
        best_logits = torch.full(
            (args_cli.num_envs, ), float("-inf"), device=device
        )
        base_obs = terrain_obs[:, :const.gait_net.robot_state_dim]

        for leg_num in range(const.robot.num_legs):
            leg_terrain_mask = FootstepCandidateSampler.filter_cost_map(
                cost_map=None,
                obs=terrain_obs,
            )
            # prior method uses 0=valid, inf=invalid, so change this to a bool mask
            leg_terrain_mask = leg_terrain_mask[:, leg_num] != float("inf")

            leg_action, leg_logit = generate_footstep_action(
                state=base_obs,
                terrain_mask=leg_terrain_mask,
                leg=leg_num,
                gaitnet=self.model,
                pos_to_idx=xy_to_idx,
                idx_to_pos=idx_to_xy,
            )
            # since we pick the single best action across all legs, we need to compare logits here
            better_mask = leg_logit.squeeze(-1) > best_logits
            best_logits = torch.where(better_mask, leg_logit.squeeze(-1), best_logits)
            # for env 0, if this is the new best action, log it
            if best_logits[0] == leg_logit.squeeze(-1)[0]:
                rospy.loginfo(f"Leg {leg_num} action: {leg_action}, logit: {leg_logit.item()}, better than best: {better_mask.item()}")
            best_actions = torch.where(
                better_mask.unsqueeze(-1),
                leg_action,
                best_actions,
            )

        # Evaluate no-op option and compare against best leg action
        # No-op one-hot is [1,0,0,0,0] (index 0), with x=0, y=0, cost=0
        no_op_one_hot = torch.zeros((args_cli.num_envs, const.robot.num_legs+1), device=device)
        no_op_one_hot[:, 0] = 1  # no-op is index 0
        no_op_obs = torch.cat(
            [
                base_obs,
                no_op_one_hot,
                torch.zeros((args_cli.num_envs, 1), device=device),  # x
                torch.zeros((args_cli.num_envs, 1), device=device),  # y
                torch.zeros((args_cli.num_envs, 1), device=device),  # cost
            ],
            dim=-1,
        )
        no_op_value, _ = self.model(no_op_obs)
        no_op_logit = no_op_value.squeeze(-1)
        
        # Check if no-op is better than best leg action
        no_op_better = no_op_logit > best_logits
        best_logits = torch.where(no_op_better, no_op_logit, best_logits)
        if no_op_better[0]:
            rospy.loginfo(f"Noop action, logit: {no_op_logit.item()}, better than best leg action: {best_logits.item()}")
        # No-op action: leg=NO_STEP (-1), x=0, y=0, duration=0
        no_op_action = torch.tensor([const.NO_STEP, 0.0, 0.0, 0.0], device=device).expand(args_cli.num_envs, -1)
        best_actions = torch.where(
            no_op_better.unsqueeze(-1),
            no_op_action,
            best_actions,
        )
        
        best_action = best_actions[0]  # since num_envs=1
        leg, x, y, duration = best_action.tolist()

        res = Res()
        if leg == const.NO_STEP:
            res.action.noop = True
            return res

        res.action.noop = False
        res.action.leg = int(leg)
        res.action.hip_offset_xy = [x, y]
        res.action.swing_duration = duration
        return res
        

    def handle_request(self, req: Req) -> Res:
        start_time = perf_counter()
        rospy.loginfo("===== Received GaitNet service request")
        input_tensor = self.interpret_request(req)

        res = self.synthesize_footstep_action(input_tensor)
        elapsed = (perf_counter() - start_time) * 1000
        rospy.loginfo(f"===== Responding with action: {pretty_result_str(res)} after {elapsed:.3f}ms\n")
        return res



if __name__ == "__main__":
    rospy.init_node("gaitnet_service")

    device_str = rospy.get_param("~device", "cpu")
    if type(device_str) is not str:
        rospy.logerr("Device parameter must be a string")
        exit(1)
    try:
        device = torch.device(device_str)
    except Exception as e:
        rospy.logerr(f"Invalid device parameter: {e}")
        exit(1)

    model_path_str = rospy.get_param("~model_path")
    if type(model_path_str) is not str:
        rospy.logerr("Model path parameter must be a string")
        exit(1)
    model_path = Path(model_path_str)
    if not model_path.exists():
        rospy.logerr(f"Model path {model_path} does not exist")
        exit(1)

    gaitnet_service = GaitNetService(device=device, model_path=model_path)
    service = rospy.Service(
        name="gaitnet_service",
        service_class=GaitNetInterface,
        handler=gaitnet_service.handle_request,
    )

    rospy.spin()
