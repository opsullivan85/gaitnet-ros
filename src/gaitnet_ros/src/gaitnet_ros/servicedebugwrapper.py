from __future__ import annotations
import rospy
from typing import Protocol
from gaitnet_interface.srv import (
    GaitNetInterfaceRequest as Req,
    GaitNetInterfaceResponse as Res
)
from gaitnet_interface.msg import StepCSpaceRobot, GaitNetInput, ControlInput, FootstepAction

class GaitnetInterfaceService(Protocol):
    def handle_request(self, req: Req) -> Res:
        ...

class ServiceDebugWrapper():
    def __init__(self, inner: GaitnetInterfaceService, namespace: str = "/debug") -> None:
        self.inner = inner
        self.control_pub = rospy.Publisher(f"{namespace}/control", ControlInput, queue_size=1)
        self.state_pub = rospy.Publisher(f"{namespace}/state", GaitNetInput, queue_size=1)
        self.step_c_space_pub = rospy.Publisher(f"{namespace}/step_c_space", StepCSpaceRobot, queue_size=1)
        self.action_pub = rospy.Publisher(f"{namespace}/action", FootstepAction, queue_size=1)

    def handle_request(self, req: Req) -> Res:
        self.control_pub.publish(req.control)
        self.state_pub.publish(req.state)
        self.step_c_space_pub.publish(req.stepCSpace)
        res = self.inner.handle_request(req)
        self.action_pub.publish(res.action)
        return res

    @classmethod
    def wrap_if_debug_service(cls, inner: GaitnetInterfaceService, namespace: str = "/debug") -> ServiceDebugWrapper | GaitnetInterfaceService:
        if rospy.get_param("~debug_service", False):
            return ServiceDebugWrapper(inner, namespace)
        return inner