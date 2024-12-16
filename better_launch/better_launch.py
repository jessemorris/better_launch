from typing import Any, Callable
import sys
import os
from ast import literal_eval
import signal
import inspect
import asyncio
from contextlib import contextmanager
from collections import deque
import logging
import osrf_pycommon.process_utils

from ament_index_python.packages import get_package_share_directory

try:
    # For anonymous nodes
    import wonderwords

    __uuid_generator = lambda g=wonderwords.RandomWord(
        exclude_with_spaces=True
    ): g.word()
except ImportError:
    import uuid

    __uuid_generator = lambda: uuid.uuid4().hex

from elements import Group, Composer, Node


__better_launch_this_defined = "__better_launch_this_defined"
__better_launch_instance = "__better_launch_instance"


def launch_this(launch_func):
    glob = globals()
    if __better_launch_this_defined in glob and __better_launch_instance not in glob:
        # Allow using launch_this only once unless we got included from another file
        raise RuntimeError("Can only use one launch decorator")

    glob[__better_launch_this_defined] = True

    # Expose launch_func args through click?
    import click

    options = []
    sig = inspect.signature(launch_func)
    for param in sig.parameters.values():
        default = param.default
        options.append(click.Option(f"--{param.name}", default=default))

    # All BL nodes and includes are async once started
    click_cmd = click.Command("main", callback=launch_func, options=options)
    try:
        click_cmd.main()
    except SystemExit as e:
        if e.code != 0:
            raise

    # Retrieve the BetterLaunch singleton
    bl = BetterLaunch()

    # LaunchService should not be run in parallel, so we instead collect all the
    # actions and then run them at the end
    if bl._deferred_ros_actions:
        from launch import LaunchDescription, LaunchService

        launch_description = LaunchDescription(bl._deferred_ros_actions)

        bl._ros2_launcher = LaunchService(noninteractive=True)
        bl._ros2_launcher.include_launch_description(launch_description)
        bl._ros2_launcher.run()


def launch_this_ros2(launch_func, to_global: bool = True):
    # Makes your main function compatible with the ros2 launch system, e.g.
    # declares arguments, creates a stub launch description, etc.
    glob = globals()
    if __better_launch_this_defined in glob and __better_launch_instance not in glob:
        # Allow using launch_this only once unless we got included from another file
        raise RuntimeError("Can only use one launch decorator")

    glob[__better_launch_this_defined] = True

    def generate_launch_description():
        from launch import LaunchDescription, LaunchContext
        from launch.actions import DeclareLaunchArgument, OpaqueFunction

        ld = LaunchDescription()

        # Declare launch arguments from the function signature
        sig = inspect.signature(launch_func)
        for param in sig.parameters.values():
            default = param.default
            if default is not inspect.Parameter.empty:
                default = str(param.default)

            ld.add_action(DeclareLaunchArgument(param.name, default_value=default))

        def ros2_wrapper(context: LaunchContext, *args, **kwargs):
            # args and kwargs are only used by OpaqueFunction when using it like partial
            launch_args = {}
            for k, v in context.launch_configurations.items():
                try:
                    launch_args[k] = literal_eval(v)
                except ValueError:
                    # Probably a string
                    # NOTE this should also make passing args to ROS2 much easier
                    # TODO should we spin our own autocomplete?
                    launch_args[k] = v

            launch_func(**launch_args)

            # Retrieve the BetterLaunch singleton
            bl = BetterLaunch()
            # Opaque functions are allowed to return additional actions
            return bl._deferred_ros_actions

        ld.add_action(OpaqueFunction(ros2_wrapper))
        return ld

    if to_global:
        glob["generate_launch_description"] = generate_launch_description

    return generate_launch_description


class _BetterLaunchMeta(type):
    # Allows reusing an already existing BetterLaunch instance.
    # Important for launch file includes.
    def __call__(cls, *args, **kwargs):
        existing_instance = globals().get(__better_launch_instance, None)
        if existing_instance is not None:
            return existing_instance

        obj = cls.__new__(cls, *args, **kwargs)
        obj.__init__(*args, **kwargs)
        return obj


class BetterLaunch(metaclass=_BetterLaunchMeta):
    def __init__(
        self,
        ns: str = "/",
        launch_args: dict = None,
        name: str = None,
        log_level: int = logging.INFO,
    ):
        if launch_args is None:
            frame = inspect.currentframe()
            try:
                # Get the caller's locals
                launch_args = {
                    k: v for k, v in frame.f_back.f_locals if not k.startswith("_")
                }
            finally:
                del frame

        if not name:
            caller = inspect.stack()[1]
            caller_module = inspect.getmodule(caller[0])
            name = os.path.basename(caller_module.__file__)
            name = os.path.splitext(name)[0]
            if name.endswith(".launch"):
                name = os.path.splitext(name)[0]

        # This signal handler only works on linux, see
        # https://github.com/ros2/launch/blob/rolling/launch/launch/launch_service.py#L213
        self.asyncio_loop = osrf_pycommon.process_utils.get_loop()
        self.asyncio_loop.add_signal_handler(self._on_sigint)
        self.asyncio_loop.add_signal_handler(self._on_sigterm)

        self.logger = logging.Logger(name, level=log_level)
        self.stdout = sys.stdout
        self.stderr = sys.stderr
        self.all_args = launch_args
        self.sigint_received = False

        if ns is None:
            ns = "/"
        elif not ns.startswith("/"):
            ns = "/" + ns

        self._group_root = Group(self, ns)
        self._group_stack = deque()
        self._group_stack.append(self._group_root)

        self._composition_node = None

        # Stores additional actions for the ROS2 launch system, 
        # e.g. includes of regular (not better-launch) launch files
        self._deferred_ros_actions = []
        self._ros2_launcher = None

    def get_unique_name(self, name: str = ""):
        return name + "_" + __uuid_generator()

    def all_groups(self):
        # Assemble all groups
        groups: list[Group] = [self._group_root]
        queue: list[Group] = [self._group_root]
        
        # Simplified breadth first search since we don't expect any loops
        while queue:
            g = queue.pop()
            groups.extend(g.children)
            queue.extend(g.children)

        return groups

    def all_nodes(self):
        nodes = []
        groups = self.all_groups()

        for g in groups:
            nodes.extend(g.nodes)
        
        return nodes

    def _on_sigint(self, signum):
        signame = signal.Signals(signum).name
        if not self.sigint_received:
            self.logger.warning(f"Received ({signame}), forwarding to child processes...")
            self.shutdown("user interrupt", signum)
            self.sigint_received = True
        else:
            self.logger.warning(f"Received ({signame}) again, ignoring")

    def _on_sigterm(self, signum):
        signame = signal.Signals(signum).name
        self.logger.error(f"Using ({signame}) can result in orphaned processes!")
        
        # Final chance for the processes to shut down
        self.shutdown(f"received ({signame})", signum)

        try:
            # Python 3.7+
            current_task = asyncio.current_task(self.asyncio_loop)
        except AttributeError:
            current_task = asyncio.Task.current_task(self.asyncio_loop)

        self.asyncio_loop.call_later(0.5, current_task.cancel)
        self.shutdown(f"received ({signame})", signum)

    def shutdown(self, reason: str, signum: int = signal.SIGTERM):
        # Tell all nodes to shut down
        for n in self.all_nodes():
            n.shutdown(reason, signum)

        # If we launched extra ROS2 actions tell the launch service to shut down, too
        if self._ros2_launcher is not None:
            self._ros2_launcher.shutdown()

    def find(self, package: str, file_name: str = None, file_dir: str = None):
        package_dir = get_package_share_directory(package) if package else None

        if file_name is None:
            return package_dir

        if package_dir is None:
            return file_name

        # do not look for it, it's (said to be) there
        if file_dir is not None:
            return os.path.join(package_dir, file_dir, file_name)

        # look for it
        for root, dirs, files in os.walk(package_dir, topdown=False):
            if file_name in files:
                return join(package_dir, root, file_name)

        # not there
        raise RuntimeError(f"Could not find file {file_name} in package {package}")

    @contextmanager
    def group(self, ns: str = None):
        group = Group(self, self._group_root, ns)
        self._group_stack[-1].add_group(group)
        self._group_stack.append(group)
        try:
            yield group
        finally:
            self._group_stack.pop()

    def node(
        self,
        pkg: str,
        exec: str,
        name: str = None,
        node_args: dict[str, Any] = None,
        *,
        anonymous: bool = False,
        hidden: bool = False,
        remap: dict[str, str] = None,
        env: dict[str, str] = None,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
        stderr_to_stdout: bool = False,
        **kwargs,
    ):
        if anonymous:
            name = self.get_unique_name(name)

        if hidden and not name.startswith("_"):
            name = "_" + name

        # TODO should the node be constructed by the group, since it makes some
        # changes to the node params?
        exec_file = self.find(pkg, exec)
        node = Node(
            self,
            exec_file,
            name,
            node_args,
            remap=remap,
            env=env,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
            emulate_tty=emulate_tty,
            stderr_to_stdout=stderr_to_stdout,
            **kwargs,
        )

        self._group_root.add_node(node)
        return node

    @contextmanager
    def compose(
        self,
        name: str,
        language: str = "cpp",
        *,
        anonymous: bool = False,
        hidden: bool = False,
        remap: dict[str, str] = None,
        env: dict[str, str] = None,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
        stderr_to_stdout: bool = False,
        **kwargs,
    ):
        if self._composition_node is not None:
            raise RuntimeError("Cannot nest composition nodes")

        if anonymous:
            name = self.get_unique_name(name)

        if hidden and not name.startswith("_"):
            name = "_" + name

        comp = Composer(
            self,
            name,
            language,
            remap=remap,
            env=env,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
            emulate_tty=emulate_tty,
            stderr_to_stdout=stderr_to_stdout,
            **kwargs,
        )

        try:
            self._group_root.add_node(comp)
            self._composition_node = comp
            yield comp
        finally:
            self._composition_node = None

    def component(
        self,
        pkg: str,
        plugin: str,
        name: str,
        *,
        anonymous: bool = False,
        hidden: bool = False,
        remap: dict[str, str] = None,
        env: dict[str, str] = None,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
        stderr_to_stdout: bool = False,
        **kwargs,
    ):
        if self._composition_node is None:
            raise RuntimeError("Cannot add component outside a compose() node")

        if anonymous:
            name = self.get_unique_name(name)

        if hidden and not name.startswith("_"):
            name = "_" + name

        self._composition_node.add_component(
            pkg,
            plugin,
            name,
            remap=remap,
            env=env,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
            emulate_tty=emulate_tty,
            stderr_to_stdout=stderr_to_stdout,
            **kwargs,
        )

    def include(self, pkg: str, launch_file: str, pass_all_args: bool = True, **kwargs):
        global_args = dict(globals())
        if pass_all_args:
            local_args = self.all_args
        else:
            local_args = {}

        if kwargs:
            local_args.update(kwargs)

        file_path = self.find(pkg, launch_file)
        if launch_file.endswith(".py"):
            with open(file_path) as f:
                content = f.read()
                if "better_launch" in content:
                    # Launch file uses better_launch, too
                    try:
                        code = compile(content, launch_file, "exec")
                        global_args["_better_launcher_instance"] = self
                        exec(code, global_args, local_args)
                        return
                    except Exception as e:
                        self.logger.error(
                            f"Launch include '{pkg}/{launch_file}' failed: {e}"
                        )
                        raise
                else:
                    # Delegate to ros2 launch service
                    from launch.actions import IncludeLaunchDescription
                    from launch.launch_description_sources import (
                        PythonLaunchDescriptionSource,
                    )

                    ros2_include = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(file_path)
                    )
                    self._deferred_ros_actions.append(ros2_include)

    # TODO Common stuff like joint_state_publisher, robot_state_publisher, moveit, rviz, gazebo
