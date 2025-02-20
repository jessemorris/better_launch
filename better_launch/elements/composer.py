from typing import Any, Callable, Mapping, Literal
import signal
import logging
from rclpy import Parameter
from composition_interfaces.srv import LoadNode, UnloadNode

from .abstract_node import AbstractNode
from .lifecycle_manager import LifecycleStage
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
        """Representation of a component, a composable object that can be loaded into a running process. Components will always use their composer's namespace.

        Note that in ROS2 launch files you can reference existing composers by name when creating components. This is a clutch because you ROS2 you can never obtain a reference to a meaningful object to refer to. Since we have actual instances in better_launch, referring to composers by name is not supported. Use :py:meth:`BetterLaunch.component` or construct your own component and pass a :py:class:`Composer` instance.

        .. seealso::

            `ROS2 About Composition <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html>`_

        Parameters
        ----------
        composer : Composer
            The composer this component will be associated with.
        package : str
            The package providing this component.
        plugin : str
            The special string that can be used for loading the component.
        name : str
            The name of the component in ROS.
        remaps : dict[str, str], optional
            Tells the node to replace any topics it wants to interact with according to the provided dict.
        params : str | dict[str, Any], optional
            Any arguments you want to provide to the node. These are the args you would typically have to declare in your launch file. A string will be interpreted as a path to a yaml file which will be lazy loaded using :py:meth:`BetterLaunch.load_params`.
        """
        super().__init__(package, plugin, name, composer.namespace, remaps, params)
        self._component_id: int = None
        self._composer = composer

    @property
    def composer(self) -> "Composer":
        """The composer this component is associated with.
        """
        return self._composer

    @property
    def component_id(self) -> int:
        """The ID this component got assigned when it was loaded into the composer. Will be None if the component is not loaded."""
        return self._component_id

    @property
    def is_loaded(self) -> bool:
        """True if the component is loaded, False otherwise."""
        return self._component_id is not None

    @property
    def plugin(self) -> str:
        """The special string that is used for loading the component. :py:meth:`executable` will return the same."""
        return self._exec

    @property
    def is_running(self) -> bool:
        if not self.is_loaded:
            return False

        return self.check_ros2_connected()

    def start(
        self,
        use_intra_process_comms: bool = True,
        **composer_extra_params,
    ) -> None:
        """Load this component into its composer.

        Additional keyword arguments will be passed as ROS parameters to the component.

        Parameters
        ----------
        use_intra_process_comms : bool, optional
            If True, the component will use intra process communication for exchanging messages with other components within the same composer.
        """
        self._component_id = self.composer.load_component(
            self,
            use_intra_process_comms=use_intra_process_comms,
            **composer_extra_params
        )

    def shutdown(self, reason: str, signum: int = signal.SIGTERM):
        """Unload this component if it was loaded.

        Parameters
        ----------
        reason : str
            A human-readable string describing why the component is being unloaded.
        signum : int, optional
            Ignored for components.
        """
        if not self.is_loaded:
            return
        
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
        """A composer is a special ROS2 node that can host other nodes (:py:class:`Component`s) within the same process, reducing overhead and enabling efficient intra process communication for message exchange.

        .. seealso::

            `ROS2 About Composition <https://docs.ros.org/en/foxy/Concepts/About-Composition.html>`_

        Parameters
        ----------
        name : str
            The name you want the composer to be known as.
        namespace : str
            The node's namespace.
        language : str, optional
            The programming language of the composer (and components) you want to use.
        composer_mode : ComposerMode, optional
            ROS2 provides special composers for components that need multithreading or should be isolated from the rest.
        component_remaps : dict[str, str], optional
            Any remaps you want to apply to all *components* loaded into this composer.
        composer_remaps : dict[str, str], optional
            Remaps you want to apply for the composer itself. Usually less useful (i.e. not at all).
        params : str | dict[str, Any], optional
            Any ROS parameters you want to pass to the composer itself. These are the args you would typically have to declare in your launch file. A string will be interpreted as a path to a yaml file which will be lazy loaded using :py:meth:`BetterLaunch.load_params`.
        cmd_args : list[str], optional
            Additional command line arguments to pass to the composer.
        env : dict[str, str], optional
            Additional environment variables to set for the composer's process.
        isolate_env : bool, optional
            If True, the composer process' env will not be inherited from the parent process. Be aware that this can result in many common things to not work anymore since e.g. keys like *PATH* will be missing.
        log_level : int, optional
            The minimum severity a logged message from this composer must have in order to be published.
        output_config : Node.LogSink  |  dict[Node.LogSource, set[Node.LogSink]], optional
            How log output from the node should be handled. Sources are `stdout`, `stderr` and `both`. Sinks are `screen`, `log`, `both`, `own_log`, and `full`. See :py:class:`Node` for more details.
        reparse_logs : bool, optional
            If True, *better_launch* will capture the composer's output and reformat it before publishing. 
        anonymous : bool, optional
            If True, the composer name will be appended with a unique suffix to avoid name conflicts.
        hidden : bool, optional
            If True, the composer name will be prepended with a "_", hiding it from common listings.
        on_exit : Callable, optional
            A function to call when the composer's process terminates (after any possible respawns).
        max_respawns : int, optional
            How often to restart the composer process if it terminates.
        respawn_delay : float, optional
            How long to wait before restarting the composer process after it terminates.
        use_shell : bool, optional
            If True, invoke the composer executable via the system shell. Use only if you know you need it.

        Raises
        ------
        ValueError
            _description_
        """
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
        """The programming language this composer (and its components) use.
        """
        return self._language

    @property
    def loaded_components(self) -> list[Component]:
        """The currently loaded components.
        """
        return list(self._loaded_components.values())

    @property
    def is_lifecycle(self) -> bool:
        """Composers are not lifecycle nodes.
        """
        return False

    def start(self) -> None:
        super().start()

        from better_launch import BetterLaunch

        launcher = BetterLaunch.instance()
        self._load_node_client = launcher.service_client(
            f"{self.fullname}/_container/load_node", LoadNode
        )
        if not self._load_node_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Failed to connect to composer load service")

        self._unload_node_client = launcher.service_client(
            f"{self.fullname}/_container/unload_node", UnloadNode
        )
        if not self._unload_node_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Failed to connect to composer unload service")

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        components = list(self._loaded_components.values())
        for comp in components:
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
        """Load this component into its composer.

        Additional keyword arguments will be passed as ROS parameters to the component. If the component is not associated with this composer yet, a warning will be logged and its association will be updated.

        Parameters
        ----------
        component: Component
            The component to load.
        use_intra_process_comms : bool, optional
            If True, the component will use intra process communication for exchanging messages with other components within the same composer.

        Returns
        -------
        int
            The ID the component got assigned by ROS.

        Raises
        ------
        ValueError
            If this composer is not running or if the component is already loaded.
        RuntimeError
            If loading the component failed.
        """
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
                for k, v in component._flat_params().items()
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
        """Unload the specified component, essentially stopping its node.

        Note that an unload request will be issued even if the component reports it is not loaded.

        Parameters
        ----------
        component : Component | int
            The component or a component's ID to stop.

        Returns
        -------
        bool
            True if unloading the component succeeded, False otherwise.

        Raises
        ------
        ValueError
            If this composer is not running.
        """
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
