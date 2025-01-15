from typing import Any, Callable
from composition_interfaces.srv import LoadNode

from .node import Node


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
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
        stderr_to_stdout: bool = False,
    ):
        # NOTE: does not support referencing an already existing composer. If you want to reuse
        # the container, just keep a reference to it.
        super().__init__(
            launcher,
            f"rcl{self.language}_components",
            "component_container",
            name,
            node_args,
            remap,
            env,
            on_exit,
            max_respawns,
            respawn_delay,
            use_shell,
            emulate_tty,
            stderr_to_stdout,
            start_immediately=True,
        )

        self.language = language
        self._load_node_client = self.launcher.ros_adapter.create_client(
            LoadNode, f"{self.name}/_container/load_node"
        )
        self._load_node_client.wait_for_service(timeout_sec=1.0)

    def add_component(self, pkg, plugin, name, **kwargs):
        # TODO handle group namespaces, remaps, etc.
        # See https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/load_composable_nodes.py
        # TODO remaps, parameters, extra arguments
        req = LoadNode.Request()
        req.package_name = pkg
        req.plugin_name = plugin
        req.node_name = name
        req.node_namespace = ...  # TODO
        req.remap_rules = {k:v for k,v in self.remap.items() if not k.startswith("_")}
        req.parameters = []  # TODO 
        req.extra_arguments = []

        # Call the load_node service
        self.logger.info(f"Loading composable node {pkg}/{plugin}...")
        # TODO make this async so that we can check for e.g. shutdown while waiting
        res = self._load_node_client.call(req)

        if res.success:
            if res.full_node_name:
                name = res.full_node_name
            self.logger.info(f"Loaded {pkg}/{plugin} as {name}")
        else:
            self.logger.error(f"Loading {pkg}/{plugin} failed: {res.error_message}")
            #raise RuntimeError(res.error_message)
