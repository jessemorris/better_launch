import os
import psutil
import signal
import re
from xml.etree import ElementTree

from ament_index_python.packages import get_packages_with_prefixes

from . import AbstractNode
from . import LiveParamsMixin


def find_node_process(namespace: str, name: str) -> list[psutil.Process]:
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

    This will first check the currently registered packages (which are stored in `$AMENT_PREFIX_PATH`). If none of these match it will search through the path starting from the end for a valid `package.xml` file. 

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
        # be sourced
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
    cmd_args: list[str], node: AbstractNode = None
) -> tuple[dict[str, str], dict[str, str], list[str]]:
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
        The ROS2 params, remaps and additional command line args.
    """
    params = {}
    remaps = {}
    additional_args = []
    is_ros_args = False
    skip = 1

    from better_launch import BetterLaunch

    bl = BetterLaunch.instance()

    for i, arg in enumerate(cmd_args):
        if skip > 0:
            # Skip the executable
            skip -= 1
            continue

        if arg.startswith("-"):
            if arg == "--ros-args":
                is_ros_args = True
                continue

            if is_ros_args:
                if arg in ["-p", "--param"]:
                    skip = 1
                    key, val = cmd_args[i + 1].split(":=")
                    params[key] = val
                    continue

                elif arg in ["-r", "--remap"]:
                    skip = 1
                    key, val = cmd_args[i + 1].split(":=")
                    if key not in ["__ns", "__node"]:
                        if ":" in key:
                            # ROS2 supports a pattern where the key is preceded by the
                            # node's name to make node-specific remaps for e.g. components
                            name, key = key.split(":", maxsplit=1)
                            if node and name != node.name:
                                continue
                        remaps[key] = val
                    continue

                elif arg == "--params-file":
                    skip = 1
                    param_file = cmd_args[i + 1]
                    params.update(
                        bl.load_params(None, param_file, node_or_namespace=node)
                    )

                else:
                    # No special handling
                    pass

        # Nothing we handle, assume it's a regular command line arg
        additional_args.append(arg)

    return params, remaps, additional_args


class ForeignNode(AbstractNode, LiveParamsMixin):
    def __init__(
        self,
        namespace: str,
        name: str,
        *,
        forward_shutdown: bool = False,
    ):
        """This class is used for representing nodes not managed by this better_launch process.

        Although ROS2 has no mechanism to retrieve e.g. the package name from a running node, most information can be retrieved from the process object itself.

        Parameters
        ----------
        namespace : str
            The node's namespace. Must be absolute, i.e. start with a '/'.
        name : str
            This node's name. If it is a ROS node it should be how the node registers with ROS.
        forward_shutdown : bool, optional
            If True, any calls to :py:meth:`shutdown` will be forwarded to the foreign node's process.
        """
        super().__init__(None, None, name, namespace, None, None)

        self.forward_shutdown = forward_shutdown
        self.cmd_args: list[str] = []

        # So far, it is not possible to get detailed information about an already running node in
        # ROS2. Even the package and executable are not available, so we have to go from the
        # process info instead.
        self._process: psutil.Process = None
        candidates = find_node_process(namespace, name)

        if not candidates:
            self.logger.warning(f"No processes matching foreign node {self.fullname}")
        elif len(candidates > 1):
            self.logger.warning(
                f"Found multiple processes matching foreign node {self.fullname}"
            )
        else:
            process = candidates[0]

            args = process.cmdline()
            exec_path = args[0]

            # Executable is easy, package has some caveats
            self._exec = os.path.basename(exec_path)
            self._pkg, _ = get_package_for_path(os.path.dirname(exec_path))

            if not self._pkg:
                self.logger.warning(
                    f"Could not determine package of foreign node {self.fullname}"
                )

            # Remaps and other command line args
            params, remaps, additional_args = parse_process_args(args, self)
            self._params = params
            self._remaps = remaps
            self.cmd_args = additional_args

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
        raise NotImplementedError("ForeignNode cannot be started")

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        if not self.is_running:
            return

        if not self.forward_shutdown:
            self.logger.info(
                "Shutdown request will NOT be forwarded to foreign process"
            )
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
