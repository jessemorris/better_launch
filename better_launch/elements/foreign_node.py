from typing import Any
import os
import psutil
import signal
import re

from ament_index_python.packages import get_packages_with_prefixes

from . import AbstractNode
from . import LiveParamsMixin


def find_node_process(namespace: str, name: str) -> list[psutil.Process]:
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
            self.logger.warning(f"Found multiple processes matching foreign node {self.fullname}")
        else:
            process = candidates[0]

            args = process.cmdline()
            exec_path = args[0]

            # Executable is easy
            self._exec = os.path.basename(exec_path)

            # Package comes with some challenges
            ros_packages = get_packages_with_prefixes()

            for pkg, pkg_path in ros_packages.items():
                # Try to find the package in the currently registered packages
                if exec_path.startswith(pkg_path):
                    self._pkg = pkg
                    break
            else:
                # Not a package currently sourced - does this ever happen?
                folder = os.path.normpath(os.path.dirname(exec_path))
                while os.pathsep in folder:
                    files = os.listdir(folder)
                    if "package.xml" in files:
                        self._pkg = os.path.basename(folder)
                        break
                    folder = folder.rsplit(os.pathsep, maxsplit=1)[0]
                else:
                    self.logger.warning(f"Could not determine package of foreign node {self.fullname}")

            # Remaps and other command line args
            params = {}
            remaps = {}
            additional_args = []
            skip = 1

            for i, arg in enumerate(args):
                if skip > 0:
                    # Skip the executable
                    skip -= 1
                    continue

                if arg.startswith("-"):
                    if arg == "--ros-args":
                        continue

                    if arg in ["-p", "--param"]:
                        skip = 1
                        key, val = args[i+1].split(":=")
                        params[key] = val
                        continue
                    
                    elif arg in ["-r", "--remap"]:
                        skip = 1
                        key, val = args[i+1].split(":=")
                        if key not in ["__ns", "__node"]:
                            remaps[key] = val
                        continue

                # Nothing we handle, assume it's a regular command line arg
                additional_args.append(arg)
            
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
            self.logger.info("Shutdown request will NOT be forwarded to foreign process")
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
