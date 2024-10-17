import asyncio
import io
import logging
import os
import platform
import signal
import traceback
from typing import Any, Callable
from osrf_pycommon.process_utils import async_execute_process
from osrf_pycommon.process_utils import AsyncSubprocessProtocol


class _ProcessProtocol(AsyncSubprocessProtocol):
    def __init__(
        self, 
        logger, 
        stdout_logger, 
        stderr_logger, 
        *, 
        output_format: str = '[{this.logger.name}] {line}', 
        cache_output: bool = True, 
        **kwargs
    ):
        super().__init__(**kwargs)
        self.pid = None
        self.logger = logger
        self.stdout_logger = stdout_logger
        self.stderr_logger = stderr_logger
        self.stdout_buffer = io.StringIO()
        self.stderr_buffer = io.StringIO()
        self.output_format = output_format

        if cache_output:
            self.on_stdout = self._on_stdout_cached
        else:
            self.on_stdout = self._on_stdout_direct

    def connection_made(self, transport):
        self.pid = transport.get_pid()
        self.logger.info(f'process started with pid [{self.pid}]')
        super().connection_made(transport)

    def on_stdout_received(self, data: bytes) -> None:
        text = data.decode(errors="replace")
        self.on_stdout(self.stdout_logger, self.stdout_buffer, text)

    def on_stderr_received(self, data: bytes) -> None:
        text = data.decode(errors="replace")
        self.on_stderr(self.stderr_logger, self.stderr_buffer, text)

    def on_process_exited(self, returncode) -> None:
        pass

    def _on_stdout_cached(self, logger, buffer, text: str) -> None:
        last_cursor = buffer.tell()
        buffer.seek(0, os.SEEK_END)  # go to end of buffer
        buffer.write(text)
        buffer.seek(last_cursor)
        new_cursor = last_cursor
        for line in buffer:
            if not line.endswith(os.linesep):
                break
            new_cursor = buffer.tell()
            logger.info(
                self.output_format.format(line=line[:-len(os.linesep)], this=self)
            )
        buffer.seek(new_cursor)

    def _on_stdout_direct(
        self, logger: logging.Logger, buffer: io.TextIOBase, text: str
    ) -> None:
        if buffer.closed:
            # buffer was probably closed by _flush_direct on shutdown. Output without buffering.
            logger.info(self.output_format.format(line=text, this=self))
        else:
            buffer.write(text)
            buffer.seek(0)
            last_line = None
            for line in buffer:
                if line.endswith(os.linesep):
                    logger.info(self.output_format.format(line=line[:-len(os.linesep)], this=self))
                else:
                    last_line = line
                    break
            buffer.seek(0)
            buffer.truncate(0)
            if last_line is not None:
                buffer.write(last_line)

    def flush_output_buffers(self):
        for line in self.stdout_buffer:
            self.stdout_logger.info(
                self.output_format.format(line=line, this=self)
            )

        for line in self.stderr_buffer:
            self.stderr_logger.info(
                self.output_format.format(line=line, this=self)
            )

        # the respawned process needs to reuse these StringIO resources,
        # close them only after receiving the shutdown
        if self.shutdown_future is None or self.shutdown_future.done():
            self.stdout_buffer.close()
            self.stderr_buffer.close()
        else:
            self.stdout_buffer.seek(0)
            self.stdout_buffer.truncate(0)
            self.stderr_buffer.seek(0)
            self.stderr_buffer.truncate(0)


async def _execute_process(context):
    if log_cmd:
        logger.info(f"Starting process '{context.cmd}' (cwd='{context.cwd}', env='{context.env}')")

    try:
        transport, subprocess_protocol = await async_execute_process(
            lambda **kwargs: _ProcessProtocol(
                context.logger, 
                context.on_stdout, 
                context.on_stderr, 
                **kwargs
            ),
            cmd=context.cmd,
            cwd=context.cwd,
            env=context.env,
            shell=context.shell,
            emulate_tty=context.emulate_tty,
            stderr_to_stdout=context.stderr_to_stdout,
        )
    except Exception:
        logger.error(f'An exception occurred while executing process:\n{traceback.format_exc()}')
        self.__cleanup()  # TODO
        return

    pid = transport.get_pid()
    await context.emit_event(ProcessStarted(**process_event_args))  # TODO
    returncode = await subprocess_protocol.complete

    if returncode == 0:
        logger.info(f'Process has finished cleanly [pid {pid}]')
    else:
        logger.error(f"Process has died [pid {pid}, exit code {returncode}, cmd '{cmd}']")
    
    await context.emit_event(ProcessExited(returncode=returncode, **process_event_args))  # TODO

    # respawn the process if necessary
    if not context.is_shutdown\  # TODO
            and context.shutdown_future is not None\  # TODO
            and not context.shutdown_future.done()\
            (max_respawns < 0 or context.respawn_retries < max_respawns):
        
        context.respawn_retries += 1  # TODO
        if respawn_delay > 0.0:
            # wait for a timeout to respawn the process
            # and handle shutdown event with future
            # to make sure `ros2 launch` exit in time
            await asyncio.wait((context.shutdown_future,), timeout=respawn_delay)

        if not context.shutdown_future.done():
            context.asyncio_loop.create_task(_execute_process(context, cmd, cwd, env, logger))  # TODO
            return

    self.__cleanup()  # TODO


class Node:
    def __init__(self, 
        launcher, 
        pkg: str, 
        exec: str,
        name: str,
        node_args: dict[str, Any] = None, 
        *, 
        env: dict[str, str] = None, 
        log_cmd: bool = True,
        on_exit: Callable = None,
        max_respawns: int = 0, 
        respawn_delay: float = 0.0, 
        use_shell: bool = False, 
        emulate_tty: bool = False, 
        stderr_to_stdout: bool = False,
    ):
        self.launcher = launcher

        self.logger = launcher.get_logger(name)
        self.stdout_logger, self.stderr_logger = launcher.get_output_loggers(name)

        self.completed_future = launcher.asyncio_loop.create_future()  # TODO
        self.shutdown_future = launcher.asyncio_loop.create_future()
        self.on_exit = on_exit

        # TODO
        path_to_pkg = ...
        cmd = ...
        cwd = ...
        env = ...

        # TODO register callbacks

    def start(self):
        if self.is_shutdown:
            self.logger.warning(f"Node {self.cmd} will not be started as the launcher has already shut down")
            return

        try:
            self.asyncio_loop.create_task(_execute_process(self))  # TODO
        except Exception:
            for event_handler in event_handlers:
                context.unregister_event_handler(event_handler)
            raise

    @property
    def is_shutdown(self):
        return self.launcher.is_shutdown()

    @property
    def is_running(self):
        # TODO
        pass
    
    # TODO all callbacks in launch_ros check whether the event matches the process, do we need that?
    def on_signal_process(self, sig):
        # TODO
        typed_event = cast(SignalProcess, context.locals.event)
        if not typed_event.process_matcher(self):
            # this event was not intended for this process
            return None

        if self.process_details is None:
            raise RuntimeError('Signal event received before execution.')
        if self._subprocess_transport is None:
            raise RuntimeError('Signal event received before subprocess transport available.')
        if self._subprocess_protocol.complete.done():
            # the process is done or is cleaning up, no need to signal
            self.logger.debug(
                "signal '{}' not set to '{}' because it is already closing".format(
                    typed_event.signal_name, self.process_details['name']),
            )
            return None

        if platform.system() == 'Windows' and sig == signal.SIGINT:
            # TODO(wjwwood): remove this when/if SIGINT is fixed on Windows
            self.logger.warning(
                "'SIGINT' sent to process[{}] not supported on Windows, escalating to 'SIGTERM'"
                .format(self.process_details['name']),
            )
            typed_event = SignalProcess(
                signal_number=signal.SIGTERM,
                process_matcher=lambda process: True)
        
        self.logger.info("sending signal '{}' to process[{}]".format(
            typed_event.signal_name, self.process_details['name']
        ))

        try:
            if sig == signal.SIGKILL:
                self._subprocess_transport.kill()  # works on both Windows and POSIX
                return None

            self._subprocess_transport.send_signal(sig)
            return None
        except ProcessLookupError:
            self.logger.debug(
                "signal '{}' not sent to '{}' because it has closed already".format(
                    typed_event.signal_name, self.process_details['name']
                )
            )

    def on_shutdown(self):
        # TODO schedule sigterm and sigkill signals
        if self.shutdown_future is None or self.shutdown_future.done():
            # Execution not started or already done, nothing to do.
            return None

        if self.completed_future is None:
            # Execution not started so nothing to do, but self.shutdown_future should prevent
            # execution from starting in the future.
            self.shutdown_future.set_result(None)
            return None

        if self.completed_future.done():
            # If already done, then nothing to do.
            self.shutdown_future.set_result(None)
            return None

        # Defer shut down if the process is scheduled to be started
        if (self.process_details is None or self._subprocess_transport is None):
            # Do not set shutdown result, as event is postponed
            context.register_event_handler(  # TODO
                OnProcessStart(
                    on_start=lambda event, context:
                    self._shutdown_process(context, send_sigint=send_sigint)))
            return None

        self.shutdown_future.set_result(None)

    def on_process_exit():
        if self.on_exit:
            self.on_exit()  # TODO
        # TODO
        flush()

    def cleanup(self):
        # Cancel any pending timers we started.
        if self.sigterm_timer is not None:  # TODO
            self.sigterm_timer.cancel()
        if self.sigkill_timer is not None:  # TODO
            self.sigkill_timer.cancel()
        
        # Close subprocess transport if any.
        if self._subprocess_transport is not None:  # TODO
            self._subprocess_transport.close()
        
        # Signal that we're done to the launch system.
        self.completed_future.set_result(None)

    
