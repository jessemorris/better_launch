from typing import Any, Callable
from .node import Node


# TODO should composer inherit from group?
class Composer(Node):
    def __init__(
        self,
        launcher,
        name: str,
        language: str,
        node_args: dict[str, Any] = None,
        *,
        remap: dict[str, str] = None,
        env: dict[str, str] = None,
        log_cmd: bool = True,
        anonymous: bool = False,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
        stderr_to_stdout: bool = False,
        autostart: bool = True,
        **kwargs,
    ):
        super().__init__(
            launcher,
            f"rcl{self.language}_components",
            "component_container",
            name,
            node_args,
            **kwargs,
        )
        # NOTE: does not support referencing an already existing instance. If you want to reuse
        # the container, just keep a reference to it.
        self.language = language

    def add_component(self, package: str, plugin: str, **kwargs):
        # TODO call the load_node service
        # TODO handle group namespaces, remaps, etc.
        # See https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/load_composable_nodes.py
        pass
