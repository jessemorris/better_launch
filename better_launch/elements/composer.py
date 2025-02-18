from typing import Any, Callable, Mapping, Literal
import signal
import logging
from rclpy import Parameter
from composition_interfaces.srv import LoadNode, UnloadNode

from .abstract_node import AbstractNode, LifecycleStage
from .node import Node


class Component(AbstractNode):
    def __init__(
        self,
        composer: "Composer",
        package: str,
        plugin: str,
        name: str,
        *,
        remaps: dict[str, str] = None,
        params: str | dict[str, Any] = None,
    ):
        super().__init__(package, plugin, name, composer.namespace, remaps, params)
        self._component_id: int = None
        self._composer = composer

    @property
    def composer(self) -> "Composer":
        return self._composer

    @property
    def component_id(self) -> int:
        return self._component_id

    @property
    def is_loaded(self) -> bool:
        return self._component_id is not None

    @property
    def plugin(self) -> str:
        return self._exec

    @property
    def is_running(self) -> bool:
        if not self.is_loaded:
            return False

        return self.is_ros2_connected

    def start(
        self,
        lifecycle_target: LifecycleStage = LifecycleStage.ACTIVE,
        use_intra_process_comms: bool = True,
        **composer_extra_args,
    ):
        self._component_id = self.composer.load_component(
            self,
            use_intra_process_comms=use_intra_process_comms,
            **composer_extra_args
        )

        if self.is_lifecycle_node:
            self.lifecycle.transition(lifecycle_target)

    def shutdown(self, reason: str, signum: int = signal.SIGTERM):
        self.logger.warning(f"Unloading component {self.name}: {reason}")
        self.composer.unload_component(self)
        self._component_id = None

    def __repr__(self) -> str:
        return f"{self.__class__.__name__} {self.package}/{self.plugin}"


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
        params: str | dict[str, Any] = None,
        cmd_args: list[str] = None,
        env: dict[str, str] = None,
        isolate_env: bool = False,
        log_level: int = logging.INFO,
        output_config: (
            Node.LogSink | dict[Node.LogSource, set[Node.LogSink]]
        ) = "screen",
        reparse_logs: bool = True,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
    ):
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
            remaps=composer_remaps,
            params=params,
            cmd_args=cmd_args,
            env=env,
            isolate_env=isolate_env,
            log_level=log_level,
            output_config=output_config,
            reparse_logs=reparse_logs,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
        )

        self._language: str = language
        # Remaps are not useful for a composable node, but we can forward them to the components
        self._component_remaps: dict[str, str] = component_remaps or {}
        self._loaded_components: dict[int, Component] = {}
        self._load_node_client = None
        self._unload_node_client = None

    @property
    def language(self) -> str:
        return self._language

    @property
    def loaded_components(self) -> list[Component]:
        return list(self._loaded_components.values())

    @property
    def is_lifecycle(self) -> bool:
        return False

    def start(self, lifecycle_target: LifecycleStage = LifecycleStage.ACTIVE) -> None:
        super().start(lifecycle_target)

        from better_launch import BetterLaunch

        launcher = BetterLaunch.wait_for_instance(0.0)
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

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        for comp in self._loaded_components.values():
            try:
                self.unload_component(comp)
            except:
                pass

        super().shutdown(reason, signum)

    def load_component(
        self,
        component: Component,
        use_intra_process_comms: bool = True,
        **composer_extra_args: dict,
    ) -> int:
        if not self.is_running:
            raise ValueError("Cannot load components into stopped composer")

        if component.is_running:
            raise ValueError("Cannot load an already component")

        if component.composer != self:
            self.logger.warning(f"Component {component.name} was created for a different Composer, updating reference")
            component._composer = self

        # Reference: https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/load_composable_nodes.py
        req = LoadNode.Request()
        req.package_name = component.package
        req.plugin_name = component.plugin
        req.node_name = component.name
        req.node_namespace = component.namespace
        req.parameters = []

        req.parameters.extend(
            [
                Parameter(name=k, value=v).to_parameter_msg()
                for k, v in component._flat_params()
            ]
        )

        remaps = dict(self._component_remaps)
        remaps.update(component.remaps)
        req.remap_rules = [f"{src}:={dst}" for src, dst in remaps.items()]

        composer_args = {}
        composer_args.update(composer_extra_args)
        composer_args["use_intra_process_comms"] = use_intra_process_comms
        req.extra_arguments = [
            Parameter(name=k, value=v).to_parameter_msg()
            for k, v in composer_args.items()
        ]

        # Call the load_node service
        res = self._load_node_client.call(req)

        if res.success:
            if res.full_node_name:
                namespace, name = res.full_node_name.rsplit("/", maxsplit=1)
                component._namespace = namespace
                component._name = name

            # Component.start() takes care of this, but the user can call load_component directly
            cid = res.unique_id
            component._component_id = cid

            self._loaded_components[cid] = component
            self.logger.info(f"Loaded component {component}")
            return cid
        else:
            self.logger.error(
                f"Loading component {component} failed: {res.error_message}"
            )
            raise RuntimeError(res.error_message)

    def unload_component(self, component: Component | int) -> bool:
        if not self.is_running:
            raise ValueError("Cannot unload components from stopped composer")

        if isinstance(component, Component):
            cid = component.component_id
        else:
            cid = component
            component = self._loaded_components[cid]

        req = UnloadNode.Request()
        req.unique_id = cid

        res = self._unload_node_client.call(req)

        if res.success:
            # Component.shutdown() takes care of this, but the user can call unload_component directly
            component._component_id = None
            del self._loaded_components[cid]
            self.logger.info(f"Unloaded component {component} ({cid})")
            return True

        self.logger.error(
            f"Unloading component {component} ({cid}) failed: {res.error_message}"
        )
        return False

    def _get_info_section_general(self):
        info = super()._get_info_section_general()
        components = "\n".join([f"  - {c.plugin}" for c in self._loaded_components.values()])
        return info + f"""
[bold]Components[/bold]
{components}
"""
