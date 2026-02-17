from __future__ import annotations
import rospy
import std_msgs.msg
import genpy
from typing import Generic, Protocol, TypeVar

ReqT = TypeVar("ReqT", bound=genpy.Message, contravariant=True)
ResT = TypeVar("ResT", bound=genpy.Message, covariant=True)

class SupportsHandleRequest(Protocol[ReqT, ResT]):
    def handle_request(self, req: ReqT) -> ResT:
        ...

class ServiceDebugWrapper(Generic[ReqT, ResT]):
    def __init__(self, inner: SupportsHandleRequest[ReqT, ResT]) -> None:
        self.inner = inner

    def handle_request(self, req: ReqT) -> ResT:
        rospy.loginfo(f"Received request: {req}")
        res = self.inner.handle_request(req)
        rospy.loginfo(f"Sending response: {res}")
        return res

    @classmethod
    def wrap_if_debug_service(cls, inner: SupportsHandleRequest[ReqT, ResT]) -> ServiceDebugWrapper[ReqT, ResT] | SupportsHandleRequest[ReqT, ResT]:
        if rospy.get_param("~debug_service", False):
            return ServiceDebugWrapper(inner)
        return inner