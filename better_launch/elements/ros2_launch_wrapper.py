import os
import signal
import inspect
import logging
import asyncio
from multiprocessing import Process, Queue
import osrf_pycommon.process_utils

import ros.logging as roslog
from utils.better_logging import PrettyFormatter, RecordForwarder, StubbornHandler
from utils.colors import get_contrast_color
from .abstract_node import AbstractNode
from .node import Node


def _launchservice_worker(
    ros2_launcher,
    q: Queue,
    logout: logging.Logger,
    logerr: logging.Logger,
    enforce_parsable_logs: bool = True,
) -> None:
    try:
        from setproctitle import setproctitle, getproctitle

        setproctitle(f"{getproctitle()} (LaunchService)")
    except ImportError:
        pass

    if enforce_parsable_logs:
        # LaunchService is a little stubborn about log formatting and always prepends the node's
        # name, but this also allows us to capture the actual source of the message
        os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "%%{severity}%%{time}%%{message}"
        os.environ["RCUTILS_COLORIZED_OUTPUT"] = "0"

        # Create an offset to avoid going through the same sequence of colors
        get_contrast_color.hue = 0.5

        # Slight of hand to capture the process' and nodes' stdout and stderr
        import launch

        std_handler = RecordForwarder()
        std_handler.add_listener(logout.handle)
        std_handler.setFormatter(
            PrettyFormatter(
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

    async def forward_launch_descriptions():
        while True:
            try:
                while not q.empty():
                    ld = q.get()
                    ros2_launcher.include_launch_description(ld)
                await asyncio.sleep(0.1)
            except Exception as e:
                logerr.info(f"Failed to add actions to ROS launch service: {e}")

    async def wrapper():
        try:
            # Execute both our forwarder and the ros2 launch service
            return asyncio.gather(
                forward_launch_descriptions(),
                ros2_launcher.run_async(shutdown_when_idle=False),
            )
        except Exception as e:
            logerr.info(f"ROS2 launch service wrapper failed: {e}")
            raise

    logout.info("Starting ROS2 launch service")

    # Basically the same as what LaunchService.run does
    loop = osrf_pycommon.process_utils.get_loop()
    task = loop.create_task(wrapper())
    while True:
        try:
            loop.run_until_complete(task)
        except Exception as e:
            logerr.info(f"LaunchService terminated: {e}")


class Ros2LaunchWrapper(AbstractNode):
    def __init__(
        self,
        process_name: str = "LaunchService",
        launch_args: list[str] = None,
        output_config: (
            Node.LogSink | dict[Node.LogSource, set[Node.LogSink]]
        ) = "screen",
        reparse_logs: bool = True,
    ):
        super().__init__(
            "ros2/launch",
            "launch_service.py",
            process_name,
            "/",
            remaps=None,
            node_args=None,
        )

        self.reparse_logs = reparse_logs
        self.output_config = output_config

        self._launch_process: Process = None
        self._action_queue = Queue()
        self._loaded_launch_descriptions = []
        self._terminate_requested = False

        # Late import to avoid making this a dependency
        import launch

        original_log_level = roslog.launch_config.level
        launch.logging.launch_config = roslog.launch_config

        self._ros2_launcher = launch.LaunchService(
            argv=launch_args, noninteractive=True
        )
        # This feels highly illegal and I love it! :>
        setattr(
            self._ros2_launcher,
            f"_{launch.LaunchService.__name__}__logger",
            self.logger,
        )

        # LaunchService modifies the log level, we restore it
        roslog.launch_config.level = original_log_level

    @property
    def pid(self) -> int:
        if self._launch_process:
            return self._launch_process.pid
        return -1

    @property
    def is_running(self) -> bool:
        return self._launch_process and self._launch_process.is_alive()

    @property
    def is_ros2_connected(self) -> bool:
        return self.is_running

    @property
    def is_lifecycle_node(self) -> bool:
        return False

    def queue_ros2_actions(self, *actions) -> None:
        import launch

        ld = launch.LaunchDescription(list(actions))
        self._loaded_launch_descriptions.append(ld)
        self._action_queue.put(ld)

    def _do_start(self) -> None:
        logout, logerr = roslog.get_output_loggers(self.name, self.output_config)

        self._launch_process = Process(
            target=_launchservice_worker,
            args=(
                self._ros2_launcher,
                self._action_queue,
                logout,
                logerr,
                self.reparse_logs,
            ),
            name=self.name,
            daemon=True,
        )
        self._launch_process.start()

    def shutdown(self, reason: str, signum: int = signal.SIGTERM) -> None:
        if not self.is_running:
            return

        if self._terminate_requested:
            # Give the process a little bit of time to terminate
            self._launch_process.join(0.5)
            if not self.is_running:
                return

        try:
            if self._terminate_requested or signum == signal.SIGKILL:
                self.logger.warning(
                    f"({reason}), but {self.name} was asked to terminate before -> escalating to SIGKILL. Killing ROS2 launch service may leave stale processes behind!"
                )
                self._launch_process.kill()
            else:
                self.logger.info(
                    f"{self.name} was asked to terminate: {reason} (SIGTERM)"
                )
                self._launch_process.terminate()
                self._terminate_requested = True
        except:
            pass

        try:
            self._launch_process.close()
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
        ld_info = "\n".join(self.describe_launch_actions())
        return f"""\
[bold]Loaded Actions[/bold]
{ld_info}
"""

    def describe_launch_actions(self) -> list[str]:
        descriptions = []

        for ld in self._loaded_launch_descriptions:
            info = "LaunchDescription("
            for entity in ld.describe_sub_entities():
                info += f"\n{self._format_ros2_entity(entity, 1)}"
            info += "\n)"

            descriptions.append(info)

        return descriptions

    def _format_ros2_entity(self, entity, depth: int = 0):
        from launch import LaunchDescriptionEntity

        indent = "  " * depth
        description = f"{indent}{entity.__class__.__name__}("
        properties = {}

        if isinstance(entity, LaunchDescriptionEntity):
            sig = inspect.signature(entity.__class__.__init__)
            # TODO look for properties, too
            for param in sig.parameters.keys():
                if param == "self":
                    continue

                val = getattr(entity, param, "<?>")
                if isinstance(entity, LaunchDescriptionEntity):
                    # TODO formatting not quite right yet
                    val = self._format_ros2_entity(val, 0)
                elif hasattr(val, "describe"):
                    val = val.describe()

                properties[param] = val

        properties_info = ", ".join(f"{k}={v}" for k, v in properties.items())
        description += f"\n{indent}  {properties_info}"

        if hasattr(entity, "describe_sub_entities"):
            for sub in entity.describe_sub_entities():
                description += f"\n{self._format_ros2_entity(sub, depth + 1)}"

        description += f"\n{indent})"
        return description
