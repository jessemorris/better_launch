from typing import Mapping, Literal
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
from concurrent.futures import Future
from pprint import pformat
from textwrap import indent

from .abstract_node import AbstractNode
from ros import logging as roslog
from utils.better_logging import PrettyFormatter


class Node(AbstractNode):
    # See ros/logging.py for details
    LogSource = Literal["stdout", "stderr", "both"]
    LogSink = Literal["screen", "log", "both", "own_log", "full"]

    def __init__(
        self,
        package: str,
        executable: str,
        name: str,
        namespace: str,
        *,
        remaps: dict[str, str] = None,
        node_args: str | dict[str, Any] = None,
        cmd_args: list[str] = None,
        env: dict[str, str] = None,
        isolate_env: bool = False,
        log_level: int = logging.INFO,
        output_config: LogSink | dict[LogSource, set[LogSink]] = "screen",
        reparse_logs: bool = True,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
    ):
        super().__init__(package, executable, name, namespace, remaps, node_args)

        self.env = env or {}
        self.isolate_env = isolate_env
        self.cmd_args = ["--log-level", logging.getLevelName(log_level)]
        if cmd_args:
            self.cmd_args.extend(cmd_args)

        self.output_config = output_config or {}
        self.reparse_logs = reparse_logs

        self.completed_future = None
        self.shutdown_future = None
        self._process: subprocess.Popen = None
        self._on_exit_callback = on_exit
        self._sigterm_timer = None
        self._sigkill_timer = None

        self._respawn_retries = 0
        self.max_respawns = max_respawns
        self.respawn_delay = respawn_delay
        self.use_shell = use_shell
        self.emulate_tty = emulate_tty

    @property
    def pid(self) -> int:
        if not self.is_running:
            return -1
        return self._process.pid

    @property
    def is_running(self) -> bool:
        return (
            self._process is not None
            and self._process.poll() is None
            and self.shutdown_future is not None
            and not self.shutdown_future.done()
            and self.completed_future is not None
            and not self.completed_future.done()
        )

    def _do_start(self) -> None:
        from better_launch import BetterLaunch

        launcher = BetterLaunch.instance()
        if launcher.is_shutdown:
            self.logger.warning(
                f"Node {self} will not be started as the launcher has already shut down"
            )
            return

        if self.is_running:
            # Already started
            self.logger.warning(f"Node {self} is already started")
            return

        self.completed_future = Future()
        self.shutdown_future = Future()

        try:
            # Note that self.process_log_level applies to the process, not our loggers
            logout, logerr = roslog.get_output_loggers(
                f"{self.name}-{self.node_id}", self.output_config
            )

            cmd = launcher.find(self.package, self.executable)
            final_cmd = [cmd] + self.cmd_args + ["--ros-args"]

            # Attach additional node args
            for key, value in self.node_args.items():
                final_cmd.extend(["-p", f"{key}:={value}"])

            # Remappings become part of the command's ros-args
            # launch_ros/actions/node.py:206
            for src, dst in self.remaps.items():
                # launch_ros/actions/node.py:481
                final_cmd.extend(["-r", f"{src}:={dst}"])

            # If an env is specified ROS2 lets it completely replace the host env. We cover this
            # through an additional flag, as often you just want to make certain overrides.
            # launch/descriptions/executable.py:199
            if self.isolate_env:
                final_env = self.env
            else:
                final_env = dict(os.environ) | self.env

            if self.reparse_logs:
                final_env["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = (
                    "%%{severity}%%{time}%%{message}"
                )
                final_env["RCUTILS_COLORIZED_OUTPUT"] = "0"

                screen_handler = roslog.launch_config.get_screen_handler()
                formatter = PrettyFormatter()
                screen_handler.setFormatterFor(logout, formatter)
                screen_handler.setFormatterFor(logerr, formatter)

            env_str = indent(pformat(self.env), "")
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
                args=[self._process, logout, logerr],
                daemon=True,
            ).start()

        except Exception:
            self.logger.error(
                f"An exception occurred while executing process:\n{traceback.format_exc()}"
            )

    def _watch_process(
        self,
        process: subprocess.Popen,
        logout: logging.Logger,
        logerr: logging.Logger,
        gather: bool = True,
    ) -> None:
        outbuf = io.StringIO()
        errbuf = io.StringIO()

        # Set the io pipes to non-blocking mode, otherwise read will wait for data
        fcntl.fcntl(process.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        fcntl.fcntl(process.stderr.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

        sel = selectors.DefaultSelector()
        sel.register(process.stdout, selectors.EVENT_READ, (logout, outbuf))
        sel.register(process.stderr, selectors.EVENT_READ, (logerr, errbuf))

        try:
            while True:
                events = sel.select()

                if not events:
                    break

                try:
                    for key, _ in events:
                        logger, buffer = key.data
                        if gather:
                            self._collect_output_bundled(key.fileobj, buffer, logger)
                        else:
                            self._collect_output_linewise(key.fileobj, buffer, logger)
                except IOError:
                    # No data available despite the selector notifying us
                    logger.error(
                        f"Failed to read process pipe, this should not happen",
                    )

            returncode = process.wait()

            if returncode == 0:
                self.logger.info(f"Process has finished cleanly [pid {process.pid}]")
            else:
                self.logger.error(
                    f"Process has died [pid {process.pid}, exit code {returncode}, cmd '{self.package}/{self.executable}']"
                )
        finally:
            if self._on_exit_callback:
                self._on_exit_callback()

            # Respawn the process if necessary
            from better_launch import BetterLaunch

            if (
                not BetterLaunch.instance().is_shutdown
                and self.shutdown_future is not None
                and not self.shutdown_future.done()
                and (self.max_respawns < 0 or self._respawn_retries < self.max_respawns)
            ):
                self.logger.info(f"Restarting {self.name} after unexpected shutdown")

                self._respawn_retries += 1
                if self.respawn_delay > 0.0:
                    time.sleep(self.respawn_delay)

                if not self.shutdown_future.done():
                    self.start()
                    return

            process.wait()
            self._on_shutdown()
            self._cleanup()

    def _collect_output_bundled(
        self, source: io.IOBase, buffer: io.StringIO, logger: logging.Logger
    ) -> None:
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
            logger.info(bundle)

    def _collect_output_linewise(
        self, source: io.IOBase, buffer: io.StringIO, logger: logging.Logger
    ) -> None:
        buffer.write(source.read())
        buffer.seek(0)
        last_line = None

        # Log every line immediately
        for line in buffer:
            if line.endswith(os.linesep):
                logger.info(line)
            else:
                last_line = line
                break

        buffer.seek(0)
        buffer.truncate(0)
        if last_line is not None:
            buffer.write(last_line)

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        signame = signal.Signals(signum).name
        self.logger.warning(
            f"{self.name} received shutdown request: {reason} ({signame})"
        )
        self._on_signal(signum)

    def _on_signal(self, signum) -> None:
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
            if signum == signal.SIGKILL:
                self._process.kill()
                return

            self._process.send_signal(signum)
        except ProcessLookupError:
            self.logger.info(
                f"'{signame}' not sent to '{self.name}' because it has closed already"
            )

    def _on_shutdown(self) -> None:
        if self.shutdown_future is None or self.shutdown_future.done():
            # Execution not started or already done, nothing to do.
            return

        if self.completed_future is None:
            # Execution not started so nothing to do, but self.shutdown_future should prevent
            # execution from starting in the future.
            self.shutdown_future.set_result(None)
            return

        if self.completed_future.done():
            # Already done, then nothing to do
            self.shutdown_future.set_result(None)
            return

        # NOTE ROS2 is also handling a case where the subprocess is only about to start
        # see if we can live without that

        self.shutdown_future.set_result(None)

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

    def _cleanup(self) -> None:
        # Cancel any pending timers we started.
        if self._sigterm_timer is not None:
            self._sigterm_timer.cancel()
        if self._sigkill_timer is not None:
            self._sigkill_timer.cancel()

        # Signal that we're done to the launch system.
        if self.completed_future is not None:
            try:
                self.completed_future.set_result(None)
            except:
                self.completed_future.cancel()

        self.my_task = None

    def _get_info_section_general(self):
        info = super()._get_info_section_general()
        return info + f"""
[bold]Process[/bold]
  PID:       {self.pid}
  Respawns:  {self._respawn_retries} / {self.max_respawns}
  Cmd Args:  {self.cmd_args}
  Env:       {self.env}
"""

    def __repr__(self) -> str:
        return f"{self.name} [node {self.node_id}, cmd {self.package}/{self.executable}, pid {self.pid}, running {self.is_running}]"
