from typing import Literal
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

from freetime.better_launch.better_launch.utils.log_formatter import RosLogFormatter


_node_counter = 0


class Node:
    def __init__(
        self,
        launcher,
        group,
        executable: str,
        name: str,
        node_args: dict[str, Any] = None,
        *,
        # TODO add to subclasses
        log_level: int = logging.INFO,
        output_config: dict[str, set[str]] = None,
        reparse_logs: bool = True,
        remap: dict[str, str] = None,
        env: dict[str, str] = None,
        isolate_env: bool = False,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
        stderr_to_stdout: bool = False,
        start_immediately: bool = True,
    ):
        self.launcher = launcher
        self.my_task = None

        self.group = group
        self.logger = group.get_logger(name)

        self.completed_future = None
        self.shutdown_future = None
        self.on_exit_callback = on_exit
        self.sigterm_timer = None
        self.sigkill_timer = None

        if not name:
            raise ValueError("Name cannot be empty")

        if not executable:
            raise ValueError("No executable provided")

        global _node_counter
        self.node_id = _node_counter
        _node_counter += 1

        self.name = name
        self.cmd = executable
        self.env = env or {}
        self.isolate_env = isolate_env
        self.node_args = node_args or {}
        self.node_args["--log-level"] = logging.getLevelName(log_level)
        self.remap = remap or {}
        # launch_ros/actions/node.py:495
        self.remap["__node"] = name

        if output_config is None:
            output_config = {"both": {"screen"}}

        self.log_level = log_level
        self.output_config = output_config
        self.reparse_logs = reparse_logs

        self._process = None

        self.respawn_retries = 0
        self.max_respawns = max_respawns
        self.respawn_delay = respawn_delay
        self.use_shell = use_shell
        self.emulate_tty = emulate_tty
        self.stderr_to_stdout = stderr_to_stdout

        self._load_node_client = None

        if start_immediately:
            self.start()

    @property
    def namespace(self):
        return self.remap.get("__ns", "/")

    @property
    def fullname(self):
        return self.namespace + "/" + self.name

    @property
    def is_running(self):
        return (
            self._process is not None
            and self.shutdown_future is not None
            and not self.shutdown_future.done()
            and self.completed_future is not None
            and not self.completed_future.done()
        )

    @property
    def is_shutdown(self):
        return self.launcher.is_shutdown

    def start(self):
        if self.is_shutdown:
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

        self.logger.info(f"Starting process '{self.cmd}' (env='{self.env}')")

        # Note that self.log_level applies to the process, not our loggers
        logout = logging.getLogger(f"{self.name}-{self.node_id}-stdout")
        self.launcher.configure_logger(
            logout, self.output_config, self.reparse_logs
        )

        logerr = logging.getLogger(f"{self.name}-{self.node_id}-stderr")
        self.launcher.configure_logger(
            logerr, self.output_config, self.reparse_logs
        )

        try:
            # Attach additional node args
            final_cmd = [self.cmd]
            for key, value in self.node_args.items():
                final_cmd.extend([key, str(value)])

            # Remappings become part of the command's ros-args
            # launch_ros/actions/node.py:206
            final_cmd.append("--ros-args")
            for src, dst in self.remap.items():
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
        finally:
            self._cleanup()

    def _watch_process(
        self,
        process: subprocess.Popen,
        logout: logging.Logger,
        logerr: logging.Logger,
        gather: bool = True,
    ):
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

                for key, _ in events:
                    logger, buffer = key.data
                    if gather:
                        try:
                            buffer.write(key.fileobj.read())
                        except IOError:
                            # No data available despite the selector notifying us
                            logger.error(
                                f"Failed to read process pipe, this should not happen",
                            )
                            break

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
                    else:
                        try:
                            buffer.write(key.fileobj.read())
                        except IOError:
                            # No data available despite the selector notifying us
                            logger.error(
                                f"Failed to read process pipe, this should not happen",
                            )
                            break

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

            returncode = process.wait()

            if returncode == 0:
                self.logger.info(f"Process has finished cleanly [pid {process.pid}]")
            else:
                self.logger.error(
                    f"Process has died [pid {process.pid}, exit code {returncode}, cmd '{self.cmd}']"
                )
        finally:
            if self.on_exit_callback:
                self.on_exit_callback()

            # Respawn the process if necessary
            if (
                not self.is_shutdown
                and self.shutdown_future is not None
                and not self.shutdown_future.done()
                and (self.max_respawns < 0 or self.respawn_retries < self.max_respawns)
            ):
                self.respawn_retries += 1
                if self.respawn_delay > 0.0:
                    time.sleep(self.respawn_delay)

                if not self.shutdown_future.done():
                    self.start()
                    return

            self._on_shutdown()
            self._cleanup()

    def shutdown(self, reason: str, signum: int = signal.SIGTERM):
        signame = signal.Signals(signum).name
        self.logger.warning(
            f"{self.name} received shutdown request: {reason} ({signame})"
        )
        self._on_signal(signum)

    def _on_signal(self, signum):
        signame = signal.Signals(signum).name

        if not self.is_running:
            # the process is done or is cleaning up, no need to signal
            self.logger.debug(
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
            self.logger.debug(
                f"'{signame}' not sent to '{self.name}' because it has closed already"
            )

    def _on_shutdown(self):
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

    def _cleanup(self):
        # Cancel any pending timers we started.
        if self.sigterm_timer is not None:
            self.sigterm_timer.cancel()
        if self.sigkill_timer is not None:
            self.sigkill_timer.cancel()

        # Signal that we're done to the launch system.
        if self.completed_future is not None:
            try:
                self.completed_future.set_result(None)
            except:
                self.completed_future.cancel()

        self.my_task = None

    def __repr__(self):
        return (
            f"{self.name} [node {self.node_id}, cmd {self.cmd}, pid {self.pid}, running {self.is_running}]"
        )
