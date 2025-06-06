from typing import Any
import os
import psutil
import signal
import re
import json
from xml.etree import ElementTree

from ament_index_python.packages import get_packages_with_prefixes

from . import AbstractNode
from . import LiveParamsMixin


def find_node_processes() -> list[psutil.Process]:
    # NOTE we won't be able to discover nodes started by ros2 run this way, but there's really
    # nothing distinctive about those, e.g.:
    #
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 run examples_rclpy_minimal_publisher publisher_local_function
    # /usr/bin/python3 /opt/ros/humble/lib/examples_rclpy_minimal_publisher/publisher_local_function
    return [p for p in psutil.process_iter() if "--ros-args" in p.cmdline()]


def find_process_for_node(namespace: str, name: str) -> list[psutil.Process]:
    """Find processes that look like ROS2 nodes which have been passed the specified namespace and name.

    Parameters
    ----------
    namespace : str
        The namespace to look for.
    name : str
        The node name to look for.

    Returns
    -------
    list[psutil.Process]
        A list processes that match the above criteria.
    """
    r_pkg = re.compile(rf"__ns:={namespace}")
    r_name = re.compile(rf"__node:={name}")

    candidates = []

    for p in psutil.process_iter():
        pkg_match = False
        name_match = False

        for arg in p.cmdline():
            if r_pkg.match(arg):
                pkg_match = True
            elif r_name.match(arg):
                name_match = True

            if pkg_match and name_match:
                candidates.append(p)
                break

    return candidates


def get_package_for_path(path: str) -> tuple[str, str]:
    """Find the ROS2 package associated with the specified path.

    This will first check the currently registered packages (which are stored in `$AMENT_PREFIX_PATH`). If none of these match it will search through the path starting from the end for a valid `package.xml` file to get the package name.

    Parameters
    ----------
    path : str
        An absolute path to find the package for.

    Returns
    -------
    tuple[str, str]
        The package name and path to the package, or (None, None) if the package could not be determined.
    """
    path = os.path.normpath(os.path.abspath(os.path.dirname(path)))
    ros_packages = get_packages_with_prefixes()

    for pkg, pkg_path in ros_packages.items():
        # Try to find the package in the currently registered packages
        if path.startswith(pkg_path):
            return pkg, pkg_path
    else:
        # Not a package currently sourced, look for a package.xml somewhere on the path. This search
        # is somewhat expensive, but we expect it to be rare since usually packages should already
        # be sourced 99.9% of the time
        while os.pathsep in path:
            # The package.xml is usually found in install/<package>/share/<package>
            package_candidate = os.path.basename(path)
            candidates = [
                os.path.join("share", package_candidate, "package.xml"),
                "package.xml",
            ]

            for package_xml in candidates:
                if os.path.isfile(package_xml):
                    # Unfortunately the package name can be different from the package's folder, so
                    # we get it from the package.xml instead
                    tree = ElementTree.parse(package_xml)
                    root = tree.getroot()
                    if root.tag != "package":
                        # Not an actual package file
                        continue

                    name_tag = root.find("name")
                    if name_tag:
                        # Found it!
                        return name_tag.text, path

            # Go up one level
            path = path.rsplit(os.pathsep, maxsplit=1)[0]

    return None, None


def parse_process_args(
    process: psutil.Process, node: AbstractNode = None
) -> tuple[str, str, dict[str, str], dict[str, str], list[str]]:
    """Parse ROS2 command line arguments and return the user-specified parts.

    In particular, this will return the passed ROS2 params, remaps and additional command line arguments. Special ROS2 arguments like `--ros-args`, `--remap`, etc. will not be included. A node may be passed in order to resolve `nodename:key:=value` style args and load parameter files from `--params-file`.

    Parameters
    ----------
    cmd_args : list[str]
        The command line arguments to parse, including the call to the executable at [0].
    node : AbstractNode, optional
        A node to use when resolving additional details.

    Returns
    -------
    tuple[dict[str, str], dict[str, str], list[str]]
        The node's namespace and name, followed by its ROS2 params, remaps and additional command line args.
    """
    from better_launch import BetterLaunch

    node_name = ""
    namespace = ""
    params = {}
    remaps = {}
    additional_args = []
    is_ros_args = False
    skip = 1

    cmd_args = process.cmdline()
    bl = BetterLaunch.instance()

    for i, arg in enumerate(cmd_args):
        if skip > 0:
            # Skip the executable and args we already parsed
            skip -= 1
            continue

        if arg == "--ros-args":
            is_ros_args = True
            continue

        if is_ros_args:
            if arg in ["-p", "--param"]:
                skip = 1
                key, val = cmd_args[i + 1].split(":=")
                params[key] = val

            elif arg in ["-r", "--remap"]:
                skip = 1
                key, val = cmd_args[i + 1].split(":=")
                if key == "__ns":
                    namespace = val
                elif key in ["__name", "__node"]:
                    node_name = val
                else:
                    if ":" in key:
                        # ROS2 supports a pattern where the key is preceded by the
                        # node's name to make node-specific remaps for e.g. components
                        name, key = key.split(":", maxsplit=1)
                        if node and name != node.name:
                            continue
                    remaps[key] = val

            elif arg == "--params-file":
                skip = 1
                param_file = cmd_args[i + 1]
                params.update(bl.load_params(None, param_file, node_or_namespace=node))

            else:
                # No special handling
                bl.logger.warning("parse_process_args: unhandled ros-arg %s", arg)
        else:
            # Nothing we handle, assume it's a regular command line arg
            additional_args.append(arg)

    return namespace, node_name, params, remaps, additional_args


def find_ros2_nodes(include_stopped: bool = False) -> list[AbstractNode]:
    """Searches the running processes for ROS2 nodes and wraps them in ForeignNode instances. Any
    processes that have been started by this better_launch process will have their appropriate
    instances returned instead (e.g. Node, Component, etc.).

    Parameters
    ----------
    include_stopped : bool, optional
        Whether to include better_launch nodes that have been created but are not running.

    Returns
    -------
    list[AbstractNode]
        A list of nodes representing discovered ROS2 processes and nodes instantiated through better_launch.
    """
    from better_launch import BetterLaunch

    bl = BetterLaunch.instance()

    # bl.query_node would iterate over all nodes every time
    bl_nodes = {n.fullname: n for n in bl.all_nodes(True, False)}
    process_nodes = [ForeignNode.wrap_process(p) for p in find_node_processes()]

    # Use ForeignNode unless we have a better representation already
    for i, node in enumerate(process_nodes):
        bln = bl_nodes.get(node.fullname, None)
        if bln:
            process_nodes[i] = bln

    if include_stopped:
        for node in bl_nodes.values():
            if node not in process_nodes:
                process_nodes.append(node)

    return process_nodes


class ForeignNode(AbstractNode, LiveParamsMixin):
    @classmethod
    def wrap_process(cls, process: psutil.Process) -> "ForeignNode":
        namespace, name, params, remaps, additional_args = parse_process_args(process)
        exec_dir = os.path.dirname(process.cmdline()[0])
        package, _ = get_package_for_path(exec_dir)

        return ForeignNode(
            process,
            package,
            name,
            namespace,
            remaps=remaps,
            params=params,
            cmd_args=additional_args,
        )

    def __init__(
        self,
        process: psutil.Process,
        package: str,
        name: str,
        namespace: str,
        *,
        remaps: dict[str, str] = None,
        params: str | dict[str, Any] = None,
        cmd_args: list[str] = None,
    ):
        """This class is used for representing nodes not managed by this better_launch process.

        Although ROS2 has no mechanism to retrieve e.g. the package name from a running node, most information can be retrieved from the process object itself.

        Parameters
        ----------
        namespace : str
            The node's namespace. Must be absolute, i.e. start with a '/'.
        name : str
            This node's name. If it is a ROS node it should be how the node registers with ROS.
        """
        super().__init__(
            package,
            process.cmdline()[0],
            name,
            namespace,
            remaps=remaps,
            params=params,
        )

        self._process = process
        self.cmd_args = cmd_args

    @property
    def pid(self) -> int:
        """The process ID of the node process. Will be -1 if the process is not running."""
        if not self.is_running:
            return -1
        return self._process.pid

    @property
    def cmd_args(self) -> list[str]:
        """Additional arguments passed to the node process."""
        return self.cmd_args

    @property
    def is_running(self) -> bool:
        return self._process and self._process.is_running()

    def start(self) -> None:
        from better_launch import BetterLaunch

        launcher = BetterLaunch.instance()
        if launcher.is_shutdown:
            self.logger.warning(
                f"Node {self} will not be started as the launcher has already shut down"
            )
            return

        if self.is_running:
            self.logger.warning(f"Node {self} is already started")
            return

        final_cmd = [self.executable] + self.cmd_args + ["--ros-args"]

        # Attach node parameters
        for key, value in self._flat_params().items():
            # Make sure the values are parseable for ROS
            final_cmd.extend(["-p", f"{key}:={json.dumps(value)}"])

        # Special args and remaps
        # launch_ros/actions/node.py:206
        for src, dst in self._ros_args().items():
            # launch_ros/actions/node.py:481
            final_cmd.extend(["-r", f"{src}:={dst}"])

        self.logger.info(f"Starting process '{' '.join(final_cmd)}'")

        # Start the node process
        # TODO Once a foreign node is restarted by us we could capture its stdout and stderr,
        # but I suspect that would be inconsistent and confusing to the user. Instead we should
        # provide a "takeover" option which replaces a foreign node with our own
        self._process = psutil.Popen(
            final_cmd,
            cwd=None,
            shell=False,
            text=True,
        )

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        if not self.is_running:
            return

        signame = signal.Signals(signum).name
        self.logger.warning(
            f"Forwarding shutdown signal to foreign process: {reason} ({signame})"
        )
        self._process.send_signal(signum)

    def _get_info_section_general(self):
        info = super()._get_info_section_general()
        return (
            info
            + f"""
[bold]Process[/bold]
  PID:       {self.pid}
  Cmd Args:  {self.cmd_args}
"""
        )
