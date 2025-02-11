import signal
import asyncio
import logging
from multiprocessing import Process, Queue
import osrf_pycommon.process_utils

import ros.logging as roslog
from .abstract_node import AbstractNode


def _launchservice_worker(logger: logging.Logger, ros2_launcher, q: Queue) -> None:
    logger.info("Starting ROS2 launch service")

    async def forward_launch_descriptions():
        while True:
            try:
                while not q.empty():
                    ld = q.get()
                    ros2_launcher.include_launch_description(ld)
                await asyncio.sleep(0.1)
            except Exception as e:
                logger.error(f"Failed to add actions to ROS launch service: {e}")

    async def wrapper():
        try:
            # Execute both our forwarder and the ros2 launch service
            return asyncio.gather(
                forward_launch_descriptions(),
                ros2_launcher.run_async(shutdown_when_idle=False),
            )
        except Exception as e:
            logger.error(f"ROS2 launch service wrapper failed: {e}")
            raise

    try:
        # Basically the same as what LaunchService.run does
        loop = osrf_pycommon.process_utils.get_loop()
        task = loop.create_task(wrapper())
        while True:
            try:
                loop.run_until_complete(task)
            except KeyboardInterrupt:
                pass
    except Exception as e:
        logger.error(f"ROS2 launch service failed to execute: {e}")
        raise


class Ros2LaunchWrapper(AbstractNode):
    def __init__(
        self,
        process_name: str = "LaunchService",
        launch_args: list[str] = None,
    ):
        super().__init__(
            "ros2/launch",
            "launch_service.py",
            process_name,
            "/",
            remaps=None,
            node_args=None,
        )

        self._launch_process: Process = None
        self._action_queue = Queue()
        self._loaded_actions = []
        self._terminate_requested = False

        # Late import to avoid making this a dependency
        import launch

        original_log_level = roslog.launch_config.level
        launch.logging.launch_config = roslog.launch_config
        self._ros2_launcher = launch.LaunchService(
            argv=launch_args, noninteractive=True
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

        self._loaded_actions.extend(actions)
        ld = launch.LaunchDescription(list(actions))
        self._action_queue.put(ld)

    def _do_start(self) -> None:
        self._launch_process = Process(
            target=_launchservice_worker,
            args=(self.logger, self._ros2_launcher, self._action_queue),
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

        if self._terminate_requested or signum == signal.SIGKILL:
            self.logger.warning(
                f"(reason), but {self.name} was asked to terminate before -> escalating to SIGKILL. Killing ROS2 launch service may leave stale processes behind!"
            )
            self._launch_process.kill()
        else:
            self.logger.info(f"{self.name} was asked to terminate: {reason} (SIGTERM)")
            self._launch_process.terminate()
            self._terminate_requested = True

        try:
            self._launch_process.close()
        except:
            pass

    def _get_info_section_general(self) -> str:
        info = super()._get_info_section_general()
        return (
            info
            + f"""\
[bold]Launch Service[/bold]
  PID:       {self.pid}
  Shutdown:  {self._ros2_launcher.context.is_shutdown}
  Args:      {self._ros2_launcher.context.argv}
  Env:       {self._ros2_launcher.context.environment}
"""
        )

    def _get_info_section_ros(self) -> str:
        # TODO does not create a nice description yet. As always, ROS2 makes it difficult
        action_info = "\n\n".join([a.describe() for a in self._loaded_actions])

        return f"""
[bold]Loaded Actions[/bold]
{action_info}
"""
