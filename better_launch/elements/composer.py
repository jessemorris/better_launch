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

    def add_component(
        self,
        pkg,
        plugin,
        name,
        remap: dict = None,
        component_args: dict = None,
        apply_composer_remaps: bool = True,
        use_intra_process_comms: bool = True,
        **extra_composer_args: dict,
    ):
        req = LoadNode.Request()
        req.package_name = pkg
        req.plugin_name = plugin
        req.node_name = name
        req.node_namespace = self.namespace
        req.parameters = [component_args] if component_args else []

        remaps = {}
        if apply_composer_remaps:
            remaps.update({k: v for k, v in self.remap.items() if not k.startswith("_")})
        if remap:
            remaps.update(remap)
        req.remap_rules = remaps
        
        composer_args = {}
        composer_args.update(extra_composer_args)
        composer_args["use_intra_process_comms"] = use_intra_process_comms
        req.extra_arguments = [composer_args]

        # Call the load_node service
        self.logger.info(f"Loading composable node {pkg}/{plugin}...")
        res = self._load_node_client.call(req)

        if res.success:
            if res.full_node_name:
                name = res.full_node_name
            self.logger.info(f"Loaded {pkg}/{plugin} as {name}")
        else:
            self.logger.error(f"Loading {pkg}/{plugin} failed: {res.error_message}")
            raise RuntimeError(res.error_message)
