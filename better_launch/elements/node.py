import os
import platform
import signal
import traceback
from typing import Any, Callable
import time
import io
import fcntl
import logging
import threading
import subprocess
import selectors
from pprint import pformat
from textwrap import indent
import json

from better_launch.utils.better_logging import LogSink
from .abstract_node import AbstractNode
from .live_params_mixin import LiveParamsMixin


class Node(AbstractNode, LiveParamsMixin):
    def __init__(
        self,
        package: str,
        executable: str,
        name: str,
        namespace: str,
        *,
        remaps: dict[str, str] = None,
        params: str | dict[str, Any] = None,
        cmd_args: list[str] = None,
        env: dict[str, str] = None,
        isolate_env: bool = False,
        log_level: int = logging.INFO,
        output: LogSink | set[LogSink] = "screen",
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        raw: bool = False,
    ):
        """An object used for starting a ROS node and capturing its output.

        Parameters
        ----------
        package : str
            The package providing the node.
        executable : str
            The executable that should be run.
        name : str, optional
            The name you want the node to be known as.
        namespace : str
            The node's namespace. Must be absolute, i.e. start with a '/'.
        remaps : dict[str, str], optional
            Tells the node to replace any topics it wants to interact with according to the provided dict.
        params : str | dict[str, Any], optional
            Any arguments you want to provide to the node. These are the args you would typically have to declare in your launch file. A string will be interpreted as a path to a yaml file which will be lazy loaded using :py:meth:`BetterLaunch.load_params`.
        cmd_args : list[str], optional
            Additional command line arguments to pass to the node.
        env : dict[str, str], optional
            Additional environment variables to set for the node's process. The node process will merge these with the environment variables of the better_launch host process unless :py:meth:`isolate_env` is True.
        isolate_env : bool, optional
            If True, the node process' env will not be inherited from the parent process and only those passed via `env` will be used. Be aware that this can result in many common things to not work anymore since e.g. keys like *PATH* will be missing.
        log_level : int, optional
            The minimum severity a logged message from this node must have in order to be published. This will be added to the cmd_args unless it is None.
        output : LogSink | set[LogSink], optional
            Determines if and where this node's output should be directed. Common choices are `screen` to print to terminal, `log` to write to a common log file, `own_log` to write to a node-specific log file, and `none` to not write any output anywhere. See :py:meth:`configure_logger` for details.
        anonymous : bool, optional
            If True, the node name will be appended with a unique suffix to avoid name conflicts.
        hidden : bool, optional
            If True, the node name will be prepended with a "_", hiding it from common listings.
        on_exit : Callable, optional
            A function to call when the node's process terminates (after any possible respawns).
        max_respawns : int, optional
            How often to restart the node process if it terminates.
        respawn_delay : float, optional
            How long to wait before restarting the node process after it terminates.
        use_shell : bool, optional
            If True, invoke the node executable via the system shell. While this gives access to the shell's builtins, this has the downside of running the node inside a "mystery program" which is platform and user dependent. Generally not advised.
        raw : bool, optional
            If True, don't treat the executable as a ROS2 node and avoid passing it any command line arguments except those specified.

        Returns
        -------
        Node
            The node object wrapping the node process.
        """
        super().__init__(
            package, executable, name, namespace, remaps, params, output=output
        )

        self.env = env or {}
        self.isolate_env = isolate_env
        self.cmd_args = cmd_args or []
        self.node_log_level = (
            logging.getLevelName(log_level) if isinstance(log_level, int) else log_level
        )
        self.use_shell = use_shell
        self.max_respawns = max_respawns
        self.respawn_delay = respawn_delay
        self._respawn_retries = 0
        self._process: subprocess.Popen = None
        self._on_exit_callback = on_exit
        self.raw = raw

    @property
    def pid(self) -> int:
        """The process ID of the node process. Will be -1 if the process is not running."""
        if not self.is_running:
            return -1
        return self._process.pid

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def join(self, timeout: float = None) -> int:
        """Wait for the underlying process to terminate and return its exit code. Returns immediately if the process is not running.

        Parameters
        ----------
        timeout : float, optional
            How long to wait for the process to finish. Wait forever if None.

        Returns
        -------
        int
            The exit code of the process, or None if it is already terminated.

        Raises
        ------
        TimeoutError
            If a timeout was specified and the process is still running by the time the timeout expires.
        """
        proc = self._process
        if proc:
            try:
                proc.wait(timeout)
            except subprocess.TimeoutExpired as e:
                raise TimeoutError from e

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

        try:
            cmd = launcher.find(
                self.package, self.executable, f"lib/**/{self.package}/"
            )

            final_cmd = [cmd]
            if self.cmd_args:
                final_cmd.extend(self.cmd_args)

            if not self.raw:
                if self.node_log_level is not None:
                    final_cmd += ["--log-level", self.node_log_level]

                final_cmd += ["--ros-args"]

                # Attach node parameters
                for key, value in self._flat_params().items():
                    # Make sure the values are parseable for ROS
                    final_cmd.extend(["-p", f"{key}:={json.dumps(value)}"])

                # Special args and remaps
                # launch_ros/actions/node.py:206
                for src, dst in self._ros_args().items():
                    # launch_ros/actions/node.py:481
                    final_cmd.extend(["-r", f"{src}:={dst}"])

            # If an env is specified ROS2 lets it completely replace the host env. We cover this
            # through an additional flag, as often you just want to make certain overrides.
            # launch/descriptions/executable.py:199
            if self.isolate_env:
                final_env = self.env
            else:
                final_env = dict(os.environ) | self.env

            env_str = indent(pformat(self.env), "")
            # All args must be strings
            final_cmd = [str(s) for s in final_cmd]

            self.logger.info(
                f"Starting process '{' '.join(final_cmd)}'\n-> env ={env_str}"
            )

            # Start the node process
            self._process = subprocess.Popen(
                final_cmd,
                cwd=None,
                env=final_env,
                shell=self.use_shell,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

            # Watch the process for output and react when it terminates
            threading.Thread(
                target=self._watch_process,
                args=[self._process],
                daemon=True,
            ).start()

        except Exception:
            self.logger.error(
                f"An exception occurred while executing process:\n{traceback.format_exc()}"
            )

    def _watch_process(
        self,
        process: subprocess.Popen,
        gather: bool = True,
    ) -> None:
        outbuf = io.StringIO()
        errbuf = io.StringIO()

        # Set the io pipes to non-blocking mode, otherwise read will wait for data
        fcntl.fcntl(process.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        fcntl.fcntl(process.stderr.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

        sel = selectors.DefaultSelector()
        sel.register(process.stdout, selectors.EVENT_READ, outbuf)
        sel.register(process.stderr, selectors.EVENT_READ, errbuf)

        try:
            while process.poll() is None:
                events = sel.select()

                if not events:
                    # events will be empty if the process received a signal
                    break

                try:
                    for key, _ in events:
                        buffer = key.data
                        if gather:
                            self._collect_output_bundled(key.fileobj, buffer)
                        else:
                            self._collect_output_linewise(key.fileobj, buffer)
                except IOError:
                    # No data available despite the selector notifying us
                    self.logger.error(
                        "Failed to read process pipe, this should not happen",
                    )

            returncode = process.wait()

            if returncode == 0:
                self.logger.warning("Process has finished cleanly")
            else:
                self.logger.critical(f"Process has died with exit code {returncode}")
            
        finally:
            if self._on_exit_callback:
                self._on_exit_callback()

            # Respawn the process if necessary
            from better_launch import BetterLaunch

            if not BetterLaunch.instance().is_shutdown and (
                self.max_respawns < 0 or self._respawn_retries < self.max_respawns
            ):
                self.logger.info(f"Restarting {self.name} after unexpected shutdown")

                self._respawn_retries += 1
                if self.respawn_delay > 0.0:
                    time.sleep(self.respawn_delay)

                # Not nice: this will run start from the watcher thread, which will then create
                # another watcher thread before this one here exits. Should be fine, just not
                # elegant.
                self.start()
            else:
                self._on_shutdown()

    def _collect_output_bundled(self, source: io.IOBase, buffer: io.StringIO) -> None:
        buffer.write(source.read())
        buffer.seek(0)
        last_line = None
        lines = []

        # Exhaust the buffer, then log whatever we collected as one message
        for line in buffer:
            if line.endswith(os.linesep):
                lines.append(line.strip(os.linesep))
            else:
                last_line = line
                break

        buffer.seek(0)
        buffer.truncate(0)
        if last_line is not None:
            buffer.write(last_line)

        bundle = "\n".join(lines)
        if bundle:
            self.logger.info(bundle)

    def _collect_output_linewise(self, source: io.IOBase, buffer: io.StringIO) -> None:
        buffer.write(source.read())
        buffer.seek(0)
        last_line = None

        # Log every line immediately
        for line in buffer:
            if line.endswith(os.linesep):
                self.logger.info(line)
            else:
                last_line = line
                break

        buffer.seek(0)
        buffer.truncate(0)
        if last_line is not None:
            buffer.write(last_line)

    def shutdown(
        self, reason: str, signum: int = signal.SIGTERM, timeout: float = 0.0
    ) -> None:
        if not self.is_running:
            return

        signame = signal.Signals(signum).name
        self.logger.warning(f"Received shutdown request: {reason} ({signame})")
        self._on_signal(signum)

        if timeout == 0.0:
            return

        try:
            self._process.wait(timeout)
        except subprocess.TimeoutExpired:
            raise TimeoutError("Node did not shutdown within the specified timeout")

    def _on_signal(self, signum) -> None:
        if not self._process or self._process.poll() is not None:
            return

        signame = signal.Signals(signum).name

        if not self.is_running:
            # the process is done or is cleaning up, no need to signal
            self.logger.info(
                f"'{signame}' not set to '{self.name}' because it is already closing"
            )
            return

        if platform.system() == "Windows" and signum == signal.SIGINT:
            # Windows doesn't handle sigterm correctly
            self.logger.warning(
                f"'SIGINT' sent to process[{self.name}] not supported on Windows, escalating to 'SIGTERM'"
            )

            signum = signal.SIGTERM
            signame = signal.SIGTERM.name

        self.logger.info(f"Sending signal '{signame}' to process [{self.name}]")

        try:
            self._process.send_signal(signum)
        except ProcessLookupError:
            self.logger.info(
                f"'{signame}' not sent to '{self.name}' because it has closed already"
            )

    def _on_shutdown(self) -> None:
        if not self.is_running:
            return

        # Send SIGTERM and SIGKILL if not shutting down fast enough
        def escalate():
            try:
                time.sleep(3.0)
                self._on_signal(signal.SIGTERM)
                time.sleep(3.0)
                self._on_signal(signal.SIGKILL)
            except Exception:
                pass

        threading.Thread(target=escalate, daemon=True).start()

    def _get_info_section_general(self):
        info = super()._get_info_section_general()
        return (
            info
            + f"""
<bold>Process</bold>
  PID:       {self.pid}
  Respawns:  {self._respawn_retries} / {self.max_respawns}
  Cmd Args:  {self.cmd_args}
  Env:       {self.env}
"""
        )

    def __repr__(self) -> str:
        return f"{self.name} [node {self.node_id}, cmd {self.package}/{self.executable}, pid {self.pid}, running {self.is_running}]"
