from typing import Any
from pathlib import Path


_node_counter = 0


class AbstractNode:
    def __init__(
        self,
        package: str,
        executable: str,
        name: str,
        namespace: str,
        node_args: dict[str, Any] = None,
        remaps: dict[str, str] = None,
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
        self._node_args = node_args or {}
        self._remaps = remaps

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
        return str(Path(self.namespace, self.name))

    @property
    def node_args(self) -> dict[str, Any]:
        return self._node_args

    @property
    def remaps(self) -> dict[str, str]:
        return self._remaps

    @property
    def is_running(self) -> bool:
        raise NotImplementedError

    def shutdown(self) -> None:
        raise NotImplementedError

    def __repr__(self):
        return __class__.__name__ + " " + self.fullname
