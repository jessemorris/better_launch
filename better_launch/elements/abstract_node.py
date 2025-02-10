from typing import Any
from pathlib import Path

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

    def start(self, lifecycle_target: LifecycleStage = LifecycleStage.ACTIVE) -> None:
        self._do_start()

        if self.is_lifecycle_node:
            self._lifecycle_manager.transition(lifecycle_target)

    def _do_start(self) -> None:
        raise NotImplementedError

    def shutdown(self, reason: str) -> None:
        raise NotImplementedError

    @property
    def is_lifecycle_node(self) -> bool:
        if self._is_lifecycle is None:
            self._is_lifecycle = LifecycleManager.is_lifecycle(self.fullname)

            if self._is_lifecycle:
                self._lifecycle_manager = LifecycleManager(self)

        return self._is_lifecycle

    @property
    def lifecycle(self) -> LifecycleManager:
        return self._lifecycle_manager

    def __repr__(self):
        return __class__.__name__ + " " + self.fullname
