from typing import Any, Callable, Mapping, Literal
import logging
from rclpy import Parameter
from composition_interfaces.srv import LoadNode, UnloadNode

from .abstract_node import AbstractNode
from .node import Node


class Component(AbstractNode):
    def __init__(
        self,
        composer: "Composer",
        component_id: int,
        package: str,
        executable: str,
        name: str,
        namespace: str,
        node_args: list[str] = None,
        remaps: dict[str, str] = None,
    ):
        super().__init__(package, executable, name, namespace, node_args, remaps)
        self._component_id = component_id
        self._composer = composer

    @property
    def component_id(self) -> int:
        return self._component_id

    @property
    def composer(self) -> "Composer":
        return self._composer

    @property
    def plugin(self) -> str:
        return self._exec

    @property
    def is_running(self) -> bool:
        from better_launch import BetterLaunch

        bl = BetterLaunch.instance()
        living_nodes = bl.shared_node.get_fully_qualified_node_names()
        return self.fullname in living_nodes

    def shutdown(self) -> None:
        self.composer.unload_component(self)

    def __repr__(self):
        return __class__.__name__ + " " + self.fullname


class Composer(Node):
    ComposerMode = Literal["normal", "multithreading", "isolated"]

    def __init__(
        self,
        name: str,
        namespace: str,
        language: str,
        composer_mode: ComposerMode = "normal",
        *,
        component_remaps: dict[str, str] = None,
        composer_remaps: dict[str, str] = None,
        node_args: str | dict[str, Any] = None,
        cmd_args: list[str] = None,
        env: dict[str, str] = None,
        log_level: int = logging.INFO,
        output_config: (
            Node.LogSink | dict[Node.LogSource, set[Node.LogSink]]
        ) = "screen",
        reparse_logs: bool = True,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
    ):
        from better_launch import BetterLaunch

        launcher = BetterLaunch.wait_for_instance(0.0)

        # NOTE: we don't support referencing an already existing composer. If you want to reuse
        # the container, just keep a reference to it.

        if composer_mode == "normal":
            executable = "component_container"
        elif composer_mode == "multithreading":
            executable = "component_container_mt"
        elif composer_mode == "isolated":
            executable = "component_container_isolated"
        else:
            raise ValueError(f"Unknown container mode '{composer_mode}")

        package = f"rcl{language}_components"

        super().__init__(
            package,
            executable,
            name,
            namespace,
            node_args=node_args,
            remaps=composer_remaps,
            cmd_args=cmd_args,
            env=env,
            log_level=log_level,
            output_config=output_config,
            reparse_logs=reparse_logs,
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
        
        self._load_node_client = launcher.service_client(
            f"{self.name}/_container/load_node", LoadNode
        )
        if not self._load_node_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Failed to connect to composer load service")

        self._unload_node_client = launcher.service_client(
            f"{self.name}/_container/unload_node", UnloadNode
        )
        if not self._unload_node_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Failed to connect to composer unload service")

    def add_component(
        self,
        pkg,
        plugin,
        name,
        *,
        remaps: dict = None,
        component_args: dict[str, Any] = None,
        use_intra_process_comms: bool = True,
        **extra_composer_args: dict,
    ) -> Component:
        # Reference: https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/load_composable_nodes.py
        req = LoadNode.Request()
        req.package_name = pkg
        req.plugin_name = plugin
        req.node_name = name
        req.node_namespace = self.namespace
        req.parameters = []

        if isinstance(component_args, str):
            from better_launch import BetterLaunch

            component_args = BetterLaunch.instance().load_params(component_args)

        if component_args:
            req.parameters.extend([
                Parameter(name=k, value=v).to_parameter_msg()
                for k, v in component_args.items()
            ])

        remaps = {}
        remaps.update(self.component_remaps)
        if remaps:
            remaps.update(remaps)
        req.remap_rules = [f"{src}:={dst}" for src, dst in remaps.items()]

        composer_args = {}
        composer_args.update(extra_composer_args)
        composer_args["use_intra_process_comms"] = use_intra_process_comms
        req.extra_arguments = [
            Parameter(name=k, value=v).to_parameter_msg()
            for k, v in composer_args.items()
        ]

        # Call the load_node service
        #self.logger.info(f"Loading composable node {pkg}/{plugin}...")
        res = self._load_node_client.call(req)

        if res.success:
            if res.full_node_name:
                name = res.full_node_name

            comp = Component(
                self,
                res.unique_id,
                pkg,
                plugin,
                name,
                self.namespace,
                component_args,
                remaps,
            )
            self.loaded_components.append(comp)
            self.logger.info(f"Loaded component {pkg}/{plugin} as {name}")
            return comp
        else:
            self.logger.error(f"Loading component {pkg}/{plugin} failed: {res.error_message}")
            raise RuntimeError(res.error_message)

    def unload_component(self, component: Component | int) -> bool:
        if isinstance(component, Component):
            cid = component.component_id
        else:
            for c in self.loaded_components:
                if c.component_id == component:
                    cid = c.component_idFalse
                    component = c
                    break

        if component not in self.loaded_components:
            self.logger.warning(f"Unloading component not belonging to this composer")

        req = UnloadNode()
        req.unique_id = cid

        #self.logger.info(f"Unloading composable node {component} ({cid})...")
        res = self._unload_node_client.call(req)

        if res.success:
            try:
                self.loaded_components.remove(component)
            except ValueError:
                pass

            self.logger.info(f"Unloaded component {component} ({cid})")
            return True
        
        self.logger.error(f"Unloading component {component} ({cid}) failed: {res.error_message}")
        return False
