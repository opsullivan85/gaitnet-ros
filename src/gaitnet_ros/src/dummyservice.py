from __future__ import annotations

from itertools import cycle
import rospy
from gaitnet_interface.srv import (
    GaitNetInterfaceRequest as Req,
    GaitNetInterfaceResponse as Res,
    GaitNetInterface,
)

class DummyService:
    def __init__(self) -> None:

        step_duration = 0.2  # seconds
        step_distance = 0.05  # meters
        step_vector = [step_distance, 0.0]

        call_rate = 25  # Hz
        num_noop = int(call_rate * step_duration) - 1

        noop = Res()
        noop.action.noop = True

        fl = Res()
        fl.action.noop = False
        fl.action.leg = fl.action.LEG_FL
        fl.action.hip_offset_xy = step_vector
        fl.action.swing_duration = step_duration

        fr = Res()
        fr.action.noop = False
        fr.action.leg = fr.action.LEG_FR
        fr.action.hip_offset_xy = step_vector
        fr.action.swing_duration = step_duration

        rl = Res()
        rl.action.noop = False
        rl.action.leg = rl.action.LEG_RL
        rl.action.hip_offset_xy = step_vector
        rl.action.swing_duration = step_duration

        rr = Res()
        rr.action.noop = False
        rr.action.leg = rr.action.LEG_RR
        rr.action.hip_offset_xy = step_vector
        rr.action.swing_duration = step_duration

        seq = []
        seq.extend([fl, rr])
        seq.extend([noop,] * num_noop)
        seq.extend([fr, rl])
        seq.extend([noop,] * num_noop)
        
        self.responses = cycle(seq)

    def handle_request(self, req: Req) -> Res:
        rospy.loginfo("="*20)
        rospy.loginfo("GOT REQUEST")
        rospy.loginfo(f"req.control: {req.control}")
        rospy.loginfo(f"req.state: {req.state}")
        rospy.logdebug(f"req.stepCSpace: {req.stepCSpace}")

        res: Res = next(self.responses)
        rospy.loginfo("RETURNING DUMMY RESPONSE")
        rospy.loginfo(f"res.action: {res.action}")
        return res


if __name__ == "__main__":
    rospy.init_node("gaitnet_service")

    dummy_service = DummyService()
    service = rospy.Service(
        name="gaitnet_service",
        service_class=GaitNetInterface,
        handler=dummy_service.handle_request,
    )

    rospy.spin()
