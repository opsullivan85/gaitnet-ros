try:
    import gaitnet
except ImportError as e:
    raise ImportError(
        "gaitnet package not found. Please ensure that gaitnet is installed and accessible." \
        "\nTry running 'pip install -e .' from `/extern/gaitnet/` (making sure the submodule is initialized)."
    ) from e

from pathlib import Path
import torch
import rospy
from gaitnet_interface.srv import (
    GaitNetInterfaceRequest as Req,
    GaitNetInterfaceResponse as Res,
    GaitNetInterface,
)

from gaitnet.gaitnet.gaitnet import GaitnetActor

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
        return torch.nn.Module()

    def handle_request(self, req: Req) -> Res:
        return Res()


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
