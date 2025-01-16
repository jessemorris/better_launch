import asyncio
import io
import os
import platform
import signal
import traceback
from typing import Any, Callable
from logging import Logger
from osrf_pycommon.process_utils import async_execute_process
from osrf_pycommon.process_utils import AsyncSubprocessProtocol


class _ProcessProtocol(AsyncSubprocessProtocol):
    def __init__(
        self,
        logger,
        output_format: str = "[{this.logger.name}] {line}",
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.pid = None
        self.logger = logger
        self.stdout_buffer = io.StringIO()
        self.stderr_buffer = io.StringIO()
        self.output_format = output_format

    def connection_made(self, transport):
        self.pid = transport.get_pid()
        self.logger.info(f"process started with pid [{self.pid}]")
        super().connection_made(transport)

    def on_stdout_received(self, data: bytes) -> None:
        text = data.decode(errors="replace")
        self._write_output(self.logger.info, self.stdout_buffer, text)

    def on_stderr_received(self, data: bytes) -> None:
        text = data.decode(errors="replace")
        self._write_output(self.logger.error, self.stderr_buffer, text)

    def on_process_exited(self, returncode) -> None:
        pass

    def _write_output(
        self, write_to_log: Callable, buffer: io.TextIOBase, text: str
    ) -> None:
        if buffer.closed:
            # buffer was probably closed by _flush_direct on shutdown. Output without buffering.
            write_to_log(self.output_format.format(line=text, this=self))
        else:
            buffer.write(text)
            buffer.seek(0)
            last_line = None
            for line in buffer:
                if line.endswith(os.linesep):
                    write_to_log(
                        self.output_format.format(
                            line=line[: -len(os.linesep)], this=self
                        )
                    )
                else:
                    last_line = line
                    break
            buffer.seek(0)
            buffer.truncate(0)
            if last_line is not None:
                buffer.write(last_line)

    def flush_output_buffers(self, finalize: bool):
        for line in self.stdout_buffer:
            self.logger.info(self.output_format.format(line=line, this=self))

        for line in self.stderr_buffer:
            self.logger.error(self.output_format.format(line=line, this=self))

        # the respawned process needs to reuse these StringIO resources,
        # close them only after receiving the shutdown
        if finalize:
            self.stdout_buffer.close()
            self.stderr_buffer.close()
        else:
            self.stdout_buffer.seek(0)
            self.stdout_buffer.truncate(0)
            self.stderr_buffer.seek(0)
            self.stderr_buffer.truncate(0)


class Node:
    def __init__(
        self,
        launcher,
        executable: str,
        name: str,
        node_args: dict[str, Any] = None,
        *,
        logger: Logger = None,
        remap: dict[str, str] = None,
        env: dict[str, str] = None,
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

        if not logger:
            logger = launcher.logger
        self.logger = logger.getChild(name)

        self.completed_future = None
        self.shutdown_future = None
        self.on_exit_callback = on_exit
        self.sigterm_timer = None
        self.sigkill_timer = None

        self.pid = -1
        self.subprocess_transport = None
        self.subprocess_protocol = None

        if not name:
            raise ValueError("Name cannot be empty")

        if not executable:
            raise ValueError("No executable provided")

        self.name = name
        self.cmd = executable
        self.env = env or {}
        self.node_args = node_args or {}
        self.remap = remap or {}
        # launch_ros/actions/node.py:495
        self.remap["__node"] = name

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
            self.subprocess_transport is not None
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
            self.logger.debug(f"Node {self} is already started")
            return

        self.completed_future = self.launcher.asyncio_loop.create_future()
        self.shutdown_future = self.launcher.asyncio_loop.create_future()
        self.my_task = self.launcher.asyncio_loop.create_task(self._execute_process())

    async def _execute_process(self):
        self.logger.info(f"Starting process '{self.cmd}' (env='{self.env}')")

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

            # If an env is specified ROS2 lets it completely replace the host env
            # launch/descriptions/executable.py:199
            if self.env:
                final_env = self.env
            else:
                final_env = dict(os.environ)

            transport, protocol = await async_execute_process(
                lambda **kwargs: _ProcessProtocol(self.logger, **kwargs),
                cmd=final_cmd,
                cwd=None,
                env=final_env,
                shell=self.use_shell,
                emulate_tty=self.emulate_tty,
                stderr_to_stdout=self.stderr_to_stdout,
            )
            self.subprocess_transport = transport
            self.subprocess_protocol = protocol
        except Exception:
            self.logger.error(
                f"An exception occurred while executing process:\n{traceback.format_exc()}"
            )
            self._cleanup()
            return

        self.pid = transport.get_pid()
        # Event: process started
        returncode = await protocol.complete

        if returncode == 0:
            self.logger.info(f"Process has finished cleanly [pid {self.pid}]")
        else:
            self.logger.error(
                f"Process has died [pid {self.pid}, exit code {returncode}, cmd '{self.cmd}']"
            )

        # Event: process terminated
        self._on_process_exit()

        # Respawn the process if necessary
        if (
            not self.is_shutdown
            and self.shutdown_future is not None
            and not self.shutdown_future.done()
            and (self.max_respawns < 0 or self.respawn_retries < self.max_respawns)
        ):
            self.respawn_retries += 1
            if self.respawn_delay > 0.0:
                # Wait for a timeout to respawn the process. The shutdown future helps to
                # shortcut the wait if required
                await asyncio.wait([self.shutdown_future], timeout=self.respawn_delay)

            if not self.shutdown_future.done():
                self.launcher.asyncio_loop.create_task(self._execute_process())
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

        if self.subprocess_protocol.complete.done():
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
                self.subprocess_transport.kill()  # works on both Windows and POSIX
                return

            self.subprocess_transport.send_signal(signum)
            return
        except ProcessLookupError:
            signame = signal.Signals(signum).name
            self.logger.debug(
                f"'{signame}' not sent to '{self.name}' because it has closed already"
            )

    def _on_process_exit(self):
        # Note that the process might be restarted. The final call will be _on_shutdown
        if self.on_exit_callback:
            self.on_exit_callback()

        if self.subprocess_transport is not None:
            self.subprocess_transport.flush_output_buffers()

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
        self.sigterm_timer = self.launcher.asyncio_loop.call_later(
            3, lambda: self._on_signal(signal.SIGTERM)
        )
        self.sigkill_timer = self.launcher.asyncio_loop.call_later(
            6, lambda: self._on_signal(signal.SIGKILL)
        )

    def _cleanup(self):
        # Cancel any pending timers we started.
        if self.sigterm_timer is not None:
            self.sigterm_timer.cancel()
        if self.sigkill_timer is not None:
            self.sigkill_timer.cancel()

        # Close subprocess transport if any.
        if self.subprocess_transport is not None:
            self.subprocess_transport.close()

        if self.subprocess_protocol is not None:
            finalize = self.shutdown_future is not None and self.shutdown_future.done()
            self.subprocess_protocol.flush_output_buffers(finalize=finalize)

        # Signal that we're done to the launch system.
        if self.completed_future is not None:
            try:
                self.completed_future.set_result(None)
            except asyncio.exceptions.InvalidStateError:
                self.completed_future.cancel("Cancelled by cleanup")

        self.my_task = None

    def __repr__(self):
        return (
            f"{self.name} [cmd {self.cmd}, pid {self.pid}, running {self.is_running}]"
        )
