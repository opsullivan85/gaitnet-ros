from pathlib import Path

import torch
import rospy

import gaitnet_interface.msg as gn_msg
from gaitnet_interface.srv import (
    GaitNetInterfaceRequest as Req,
    GaitNetInterfaceResponse as Res,
    GaitNetInterface,
)



class GaitNetService:
    def __init__(self, device: str, model_path: Path) -> None:
        pass

    def handle_request(self, req: Req) -> Res:

        rospy.loginfo(req)
        req.stepCSpace.FL.legCSpace
        return Res()  # Placeholder response

if __name__ == "__main__":
    rospy.init_node("gaitnet_service")

    device_str = rospy.get_param("~device", "cpu")
    if type(device_str) is not str:
        rospy.logerr("Device parameter must be a string")
        exit(1)
    device = torch.device(device_str)

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
