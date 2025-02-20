from typing import Any
import os
import signal
import logging
import asyncio
import threading
from multiprocessing import Process, Queue
import osrf_pycommon.process_utils
from setproctitle import setproctitle, getproctitle

import better_launch.ros.logging as roslog
from better_launch.utils.better_logging import PrettyLogFormatter, RecordForwarder, StubbornHandler
from better_launch.utils.colors import get_contrast_color
from .abstract_node import AbstractNode
from .node import Node


def _launchservice_worker(
    name: str,
    launchservice_args: list[Any],
    launch_action_queue: Queue,
    log_queue: Queue,
) -> None:
    """This function will run in a child process and will not have access to any objects already in memory UNLESS they are passed to it as arguments. See the comments for further details.
    """
    # Makes it easier to tell what's going on in the process table
    setproctitle(f"{getproctitle()} ({name})")

    # Late import to avoid adding ROS2 launch as a dependency - we are committed here!
    import launch

    # LaunchService is a little stubborn about log formatting and always prepends the node's
    # name and then appends the output format, but this also allows us to capture the actual 
    # source of the message
    os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "%%{severity}%%{time}%%{message}"
    os.environ["RCUTILS_COLORIZED_OUTPUT"] = "0"

    # Create an offset to avoid going through the same sequence of colors as the host process
    get_contrast_color.hue = 0.3

    def handle_record(record: logging.LogRecord) -> None:
        log_queue.put(record)

    std_handler = RecordForwarder()
    std_handler.add_listener(handle_record)
    std_handler.setFormatter(
        PrettyLogFormatter(
            roslog_pattern=r"\[(.+)] *%%(\w+)%%([\d.]+)%%(.*)",
            pattern_info=["name", "levelname", "created", "msg"],
            color_per_source=True,
        )
    )

    # The ROS2 launch system will set new formatters for each node and source, so our formatter
    # wouldn't be used. We either have to make our formatter more stubborn, or we run ROS2
    # in a proper subprocess and create a small launch file to load our actions. However,
    # there is no easy way to serialize a launch description...
    launch.logging.launch_config.screen_handler = StubbornHandler(std_handler)
    logger = launch.logging.get_logger(name)

    launch_service = launch.LaunchService(argv=launchservice_args, noninteractive=True)
    
    # This feels highly illegal and I love it! :>
    setattr(
        launch_service,
        f"_{launch_service.__class__.__name__}__logger",
        logger,
    )

    # We want to keep an eye on the pipe for new LaunchDescriptions to load
    async def forward_launch_descriptions():
        while True:
            try:
                while not launch_action_queue.empty():
                    ld = launch_action_queue.get()
                    launch_service.include_launch_description(ld)
                await asyncio.sleep(0.1)
            except Exception as e:
                logger.error(f"Failed to add actions to ROS launch service: {e}")

    async def wrapper():
        try:
            # Execute both our forwarder and the ros2 launch service
            return asyncio.gather(
                forward_launch_descriptions(),
                launch_service.run_async(shutdown_when_idle=False),
            )
        except Exception as e:
            logger.error(f"ROS2 launch service wrapper failed: {e}")
            raise

    logger.info("Starting ROS2 launch service")

    # Basically the same as what LaunchService.run does
    loop = osrf_pycommon.process_utils.get_loop()
    task = loop.create_task(wrapper())
    while True:
        try:
            loop.run_until_complete(task)
        except Exception as e:
            logger.warning(f"ROS2 launch service terminated: {e}")


class Ros2LaunchWrapper(AbstractNode):
    def __init__(
        self,
        name: str = "LaunchService",
        launchservice_args: list[str] = None,
        output_config: (
            Node.LogSink | dict[Node.LogSource, set[Node.LogSink]]
        ) = "screen",
    ):
        """Hosts a separate process running a ROS2 `LaunchService` instance (the main entrypoint of the ROS2 launch system). 
        
        Note that although the ROS2 launch service may start an arbitrary number of nodes they will not be accessible for interaction beyond showing log output. Output on stdout and stderr from the process (and its nodes) will be captured, reformatted and separated by source.
        
        While this is a node-like object, it does not represent a node in ROS. This was done so that e.g. the TUI can be used to interact with the ROS2 launch system to e.g. include regular ROS2 launch files. Running a full process and a separate launch system is fairly resource heavy, however, `LaunchService` insists on running on the main thread.

        .. seealso::

            `ROS2 LaunchService <https://github.com/ros2/launch/blob/rolling/launch/launch/launch_service.py>`_

        Parameters
        ----------
        name : str, optional
            The name that will be used for the logger and the child process.
        launchservice_args : list[str], optional
            Additional arguments to pass to the ROS2 launch service. These will show up in the ROS2 `LaunchContext`.
        output_config : Node.LogSink  |  dict[Node.LogSource, set[Node.LogSink]], optional
            How log output from the launch service should be handled. Sources are `stdout`, `stderr` and `both`. Sinks are `screen`, `log`, `both`, `own_log`, and `full`.
        """
        super().__init__(
            "ros2/launch",
            "launch_service.py",
            name,
            "/",
            remaps=None,
            params=None,
        )

        self.output_config = output_config
        self._launchservice_args = launchservice_args

        self._process: Process = None
        self._launch_action_queue = Queue()
        self._process_log_queue = Queue()
        self._loaded_launch_descriptions = []
        self._terminate_requested = False

    @property
    def pid(self) -> int:
        """The process ID of the node process. Will be -1 if the process is not running.
        """
        if self._process:
            return self._process.pid
        return -1

    @property
    def launchservice_args(self) -> list[str]:
        """Additional arguments that are passed to the launch service.
        """
        return self._launchservice_args

    @property
    def is_running(self) -> bool:
        try:
            return self._process and self._process.is_alive()
        except ValueError:
            # For some reason we can't call is_alive when the process was closed
            return False

    def check_ros2_connected(self, timeout: float = None) -> bool:
        """Equal to :py:meth:`is_running` for this class.
        """
        return self.is_running

    def check_lifecycle_node(self, timeout: float = None) -> bool:
        """This is never a lifecycle node.
        """
        return False

    def queue_ros2_actions(self, *actions) -> None:
        """Add ROS2 actions that will be loaded asynchronously by the launch service once it is running. Actions are bundled as a `LaunchDescription` before sending them off.
        """
        import launch

        ld = launch.LaunchDescription(list(actions))
        self._launch_action_queue.put(ld)
        self._loaded_launch_descriptions.append(ld)

    def start(self) -> None:
        if self.is_running:
            self.logger.warning(f"LaunchService {self.name} is alrady running")
            return

        logout, logerr = roslog.get_output_loggers(self.name, self.output_config)

        # Note that passing loggers will not work for the TUI, as they would have to communicate
        # across the process boundaries. In general, only basic values and instances from the 
        # multiprocessing module should be passed to the process
        self._process = Process(
            target=_launchservice_worker,
            args=(
                self.name,
                self.launchservice_args,
                self._launch_action_queue,
                self._process_log_queue,
            ),
            name=self.name,
            daemon=True,
        )
        self._process.start()

        threading.Thread(
            target=self._process_watcher, args=(logout, logerr), daemon=True
        ).start()

    def _process_watcher(self, logout: logging.Logger, logerr: logging.Logger):
        q = self._process_log_queue

        while self.is_running:
            try:
                while not q.empty():
                    record: logging.LogRecord = q.get()
                    logout.handle(record)
            except Exception as e:
                logerr.info(f"Receiving log record failed: {e}")

        self._process.close()

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        if not self.is_running:
            return

        if self._terminate_requested:
            # Give the process a little bit of time to terminate
            self._process.join(0.5)
            if not self.is_running:
                return

        try:
            if self._terminate_requested or signum == signal.SIGKILL:
                # TODO seems to always happen, at least with the TUI?
                self.logger.warning(
                    f"({reason}), but {self.name} was asked to terminate before -> escalating to SIGKILL. Killing ROS2 launch service may leave stale processes behind!"
                )
                self._process.kill()
            else:
                self.logger.info(
                    f"{self.name} was asked to terminate: {reason} (SIGTERM)"
                )
                self._process.terminate()
                self._terminate_requested = True
        except:
            pass

    def _get_info_section_general(self) -> str:
        return (
            super()._get_info_section_general()
            + f"""\
[bold]Launch Service[/bold]
  PID:       {self.pid}
"""
        )

    def _get_info_section_ros(self) -> str:
        return f"""\
[bold]Loaded Actions[/bold]
{self.describe_launch_actions()}
"""

    def describe_launch_actions(self) -> str:
        """Returns a best-effort summary of all `LaunchDescription`s that have been loaded so far.

        Since ROS2 does not provide a way to serialize its slew of launch actions (beyond *maybe* pickle), it is not possible to create a full representation without special handlers. However, *most* launch actions expose their relevant data as properties. This function will look for these and recurse into them if they return launch actions.

        Returns
        -------
        str
            A formatted and indented representation of the launch descriptions scheduled for loading thus far.
        """
        descriptions = []

        for idx, ld in enumerate(self._loaded_launch_descriptions):
            info = f"({idx}) LaunchDescription(["
            for entity in ld.describe_sub_entities():
                info += "\n  " + self._format_properties(entity, 1) + ","
            info += "\n])"

            descriptions.append(info)

        return "\n".join(descriptions)

    def _format_properties(self, entity, depth: int = 0):
        indent = "    "
        description = f"{entity.__class__.__name__}("

        for param in dir(entity.__class__):
            if param == "self":
                continue

            if param.startswith("_"):
                continue

            val = getattr(type(entity), param)
            if not isinstance(val, property):
                continue

            val = val.fget(entity)

            # TODO some substitutions like LaunchDescriptionSource will only be resolved after
            # they have been executed by ROS2 launch, but we need to do this on the side of the
            # process, NOT in the host process where they are never run
            if val is None:
                val = "None"
            elif val.__class__.__module__.startswith("launch"):
                val = self._format_properties(val, depth + 1)
            elif isinstance(val, str):
                val = "'" + val + "'"

            description += f"\n{indent * (depth + 1)}{param} = {val},"

        description += f"\n{indent * depth})"
        return description
