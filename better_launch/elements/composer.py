from typing import Any, Callable, Mapping, Literal
import logging
from rclpy import Parameter
from composition_interfaces.srv import LoadNode

from .node import Node


class Composer(Node):
    ComposerMode = Literal["normal", "multithreading", "isolated"]

    def __init__(
        self,
        launcher,
        name: str,
        language: str,
        composer_mode: ComposerMode = "normal",
        component_remaps: dict[str, str] = None,
        *,
        log_level: int = logging.INFO,
        output_config: (
            Node.LogSink | dict[Node.LogSource, set[Node.LogSink]]
        ) = "screen",
        reparse_logs: bool = True,
        composer_remaps: dict[str, str] = None,
        env: dict[str, str] = None,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
    ):
        # NOTE: we don't support referencing an already existing composer. If you want to reuse
        # the container, just keep a reference to it.

        if composer_mode == "normal":
            container = "component_container"
        elif composer_mode == "multithreading":
            container = "component_container_mt"
        elif composer_mode == "isolated":
            container = "component_container_isolated"
        else:
            raise ValueError(f"Unknown container mode '{composer_mode}")

        executable = launcher.find(f"rcl{language}_components", container)

        super().__init__(
            launcher,
            executable,
            name,
            node_args=None,
            cmd_args=None,
            log_level=log_level,
            output_config=output_config,
            reparse_logs=reparse_logs,
            remap=composer_remaps,
            env=env,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
            emulate_tty=emulate_tty,
            start_immediately=True,
        )

        self.language = language
        # Remaps are not useful for a composable node, but we can forward them to the components
        self.component_remaps = component_remaps or {}
        self.loaded_components = []
        self._load_node_client = self.launcher.service_client(
            f"{self.name}/_container/load_node", LoadNode
        )
        if not self._load_node_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Failed to connect to composer load service")

    def add_component(
        self,
        pkg,
        plugin,
        name,
        component_args: dict[str, Any] = None,
        *,
        remap: dict = None,
        apply_composer_remaps: bool = True,
        use_intra_process_comms: bool = True,
        **extra_composer_args: dict,
    ):
        # Reference: https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/load_composable_nodes.py
        req = LoadNode.Request()
        req.package_name = pkg
        req.plugin_name = plugin
        req.node_name = name
        req.node_namespace = self.namespace
        req.parameters = []

        if isinstance(component_args, str):
            component_args = self.launcher.load_params(component_args)

        if component_args:
            req.parameters.append(component_args)

        remaps = {}
        if apply_composer_remaps:
            remaps.update(self.component_remaps)
        if remap:
            remaps.update(remap)
        req.remap_rules = [f"{src}:={dst}" for src, dst in remaps.items()]

        composer_args = {}
        composer_args.update(extra_composer_args)
        composer_args["use_intra_process_comms"] = use_intra_process_comms
        req.extra_arguments = [
            Parameter(name=k, value=v).to_parameter_msg() 
            for k,v in composer_args.items()
        ]

        # Call the load_node service
        self.logger.info(f"Loading composable node {pkg}/{plugin}...")
        res = self._load_node_client.call(req)

        if res.success:
            if res.full_node_name:
                name = res.full_node_name
            self.loaded_components.append(f"{name} ({plugin})")
            self.logger.info(f"Loaded {pkg}/{plugin} as {name}")
        else:
            self.logger.error(f"Loading {pkg}/{plugin} failed: {res.error_message}")
            raise RuntimeError(res.error_message)
