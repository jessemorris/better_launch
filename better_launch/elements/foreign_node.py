from typing import Any
import signal

from . import AbstractNode
from . import LiveParamsMixin


class ForeignNode(AbstractNode, LiveParamsMixin):
    def __init__(
        self,
        namespace: str,
        name: str,
        package: str = "<unknown>",
        executable: str = "<unknown>",
    ):
        """This class is used for representing nodes not managed by this better_launch process.

        Did you notice the different order of arguments compared to other node implementations? So far, it is not possible to get detailed information about an already running node in ROS2. Even the package and executable are hidden, so the only things we can know and interact with right now are topics, services and params.

        Parameters
        ----------
        namespace : str
            The node's namespace. Must be absolute, i.e. start with a '/'.
        name : str
            This node's name. If it is a ROS node it should be how the node registers with ROS.
        package : str, optional
            The package this node can be found in.
        executable : str, optional
            How the node can be executed. Not necessarily an executable file object.
        """
        # TODO check the actual ROS2 low level implementation if we can find out the package and 
        # executable somehow
        super().__init__(package, executable, name, namespace, None, None)

    @property
    def is_running(self) -> bool:
        return self.check_ros2_connected()

    @property
    def params(self) -> dict[str, Any]:
        return {}

    def start(self) -> None:
        raise NotImplementedError("Not supported by ForeignNode")

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        raise NotImplementedError("Not supported by ForeignNode")
