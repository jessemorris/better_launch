from typing import Any
import signal

import ros.logging as roslog
from .lifecycle_manager import LifecycleManager, LifecycleStage


_node_counter = 0


class AbstractNode:
    def __init__(
        self,
        package: str,
        executable: str,
        name: str,
        namespace: str,
        remaps: dict[str, str] = None,
        node_args: str | dict[str, Any] = None,
    ):
        if not name:
            raise ValueError("Name cannot be empty")

        if not executable:
            raise ValueError("No executable provided")

        if remaps is None:
            remaps = {}

        if not namespace and "__ns" not in remaps:
            raise ValueError("Namespace not defined")

        if namespace and "__ns" in remaps and remaps["__ns"] != namespace:
            raise ValueError("Conflicting namespace definitions")

        if not namespace:
            namespace = remaps["__ns"]

        # Why do I hear mad hatter music???
        # See launch_ros/actions/node.py:495
        remaps["__ns"] = namespace
        remaps["__node"] = name

        global _node_counter
        self.node_id = _node_counter
        _node_counter += 1

        self._pkg = package
        self._exec = executable
        self._name = name
        self._namespace = namespace
        self._remaps = remaps
        self._node_args = node_args or {}
        self._is_lifecycle: bool = None
        self._lifecycle_manager: LifecycleManager = None

        self.logger = roslog.get_logger(self.fullname)

    @property
    def package(self) -> str:
        return self._pkg

    @property
    def executable(self) -> str:
        return self._exec

    @property
    def name(self) -> str:
        return self._name

    @property
    def namespace(self) -> str:
        return self._namespace

    @property
    def fullname(self) -> str:
        ns = self.namespace.strip("/")
        if not ns:
            return "/" + self.name
        return "/" + ns + "/" + self.name

    @property
    def node_args(self) -> dict[str, Any]:
        if isinstance(self._node_args, str):
            from better_launch import BetterLaunch

            bl = BetterLaunch.instance()
            if not bl:
                return self._node_args

            self._node_args = bl.load_params(self._node_args, self)

        return self._node_args

    @property
    def remaps(self) -> dict[str, str]:
        return self._remaps

    @property
    def is_running(self) -> bool:
        raise NotImplementedError

    @property
    def is_ros2_connected(self) -> bool:
        from better_launch import BetterLaunch

        bl = BetterLaunch.instance()
        if not bl:
            return None

        # Check if the node shows up in the list of running ROS nodes
        try:
            living_nodes = [
                ns + ('' if ns.endswith('/') else '/') + name
                for name, ns in bl.shared_node.get_node_names_and_namespaces()
            ]
            return self.fullname in living_nodes
        except:
            # Cannot check if the shared node was shut down
            return None

    def start(self, lifecycle_target: LifecycleStage = LifecycleStage.ACTIVE) -> None:
        if self.is_running:
            return
        
        self._do_start()

        # TODO wait for node to come up fully
        if self.is_lifecycle_node:
            self._lifecycle_manager.transition(lifecycle_target)

    def _do_start(self) -> None:
        raise NotImplementedError

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        raise NotImplementedError

    @property
    def is_lifecycle_node(self) -> bool:
        if self._is_lifecycle is None:
            self._is_lifecycle = LifecycleManager.is_lifecycle(self)

            if self._is_lifecycle:
                self._lifecycle_manager = LifecycleManager(self)

        return self._is_lifecycle

    @property
    def lifecycle(self) -> LifecycleManager:
        # Returning the _lifecycle_manager would be fine, but this way it will be instantiated 
        # if supported and not yet done before
        if self.is_lifecycle_node:
            return self._lifecycle_manager
        return None

    def get_info_sheet(self) -> str:
        # ROS2 prints a lot of useless stuff and avoids the things that are interesting most of
        # the time, like who is actually subscribed where. Let's fix this!
        return "\n".join([
            self._get_info_section_general(),
            self._get_info_section_config(),
            self._get_info_section_ros(),
        ])

    def _get_info_section_general(self) -> str:
        return f"""\
[bold]{self.name} ({self.__class__.__name__})[/bold]
  Status:    {'[green]alive[/green]' if self.is_running else '[red]dead[/red]'}
  Lifecycle: {self.lifecycle.current_stage.name if self.is_lifecycle_node else 'None'}
  Package:   {self.package}
  Command:   {self.executable}
  Namespace: {self.namespace}
"""

    def _get_info_section_config(self) -> str:
        return f"""\
[bold]Config[/bold]
  Node Args: {self.node_args}
  Remaps:    {self.remaps}
"""

    def _get_info_section_ros(self) -> str:
        if self.is_ros2_connected:
            from better_launch import BetterLaunch

            shared_node = BetterLaunch.instance().shared_node

            # Topics the node is publishing
            pubs = shared_node.get_publisher_names_and_types_by_node(
                self.name, self.namespace
            )
            pubs.sort()
            pubs_text = ""
            for topic, types in pubs:
                pubs_text += f"\n  {topic} [{', '.join(types)}]"

            # Topics the node is subscribed to
            subs = shared_node.get_subscriber_names_and_types_by_node(
                self.name, self.namespace
            )
            subs.sort()
            subs_text = ""
            for topic, types in subs:
                subs_text += f"\n  {topic} [{', '.join(types)}]"

            # Provided services
            services = shared_node.get_service_names_and_types_by_node(
                self.name, self.namespace
            )
            services.sort()
            services_text = ""
            for srv, types in services:
                services_text += f"\n  {srv} [{', '.join(types)}]"
        else:
            pubs_text = ""
            subs_text = ""
            services_text = ""

        return f"""\
[bold]Publishers:[/bold] {pubs_text}

[bold]Subscriptions:[/bold] {subs_text}

[bold]Services:[/bold] {services_text}
"""

    def __repr__(self):
        return __class__.__name__ + " " + self.fullname
