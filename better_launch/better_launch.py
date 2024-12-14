from typing import Any, Callable
import sys
import os
import signal
import inspect
import asyncio
from contextlib import contextmanager
from collections import deque
import logging

try:
    # For anonymous nodes
    import wonderwords
    __uuid_generator = lambda g=wonderwords.RandomWord(exclude_with_spaces=True): g.word()
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
        raise RuntimeError("@launch_this can only be used once")

    glob[__better_launch_this_defined] = True

    # TODO allow using this only once
    import click

    sig = inspect.signature(launch_func)
    options = {}

    for key, param in sig.parameters.items():
        default = param.default
        if default is inspect.Parameter.empty:
            default = None

        options[key] = click.Option(f"--{key}", default=default)

    # TODO launch automatically
    # TODO enable autocomplete
    cmd = click.Command(callback=launch_func, params=options)
    return cmd


def launch_as_ros(launch_func, to_global: bool = True):
    # TODO should make the function compatible with the ros2 launch system, e.g.
    # declare arguments, create a stub launch description, etc. Also should expose
    # some kind of interface so that it can be included by other better_launch
    # scripts.
    def generate_launch_description(_better_launch_instance=None):
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument

        ld = LaunchDescription()

        # Declare launch arguments from the function signature
        sig = inspect.signature(launch_func)
        for param in sig.parameters:
            default = None
            if default is not Parameter.empty:
                default = str(param.default)

            ld.add_action(DeclareLaunchArgument(param.name, default_value=default))

        # TODO custom action to execute launch_func and pass it the declared arguments
        ld.add_action(...)
        return ld

    if to_global:
        globals()["generate_launch_description"] = generate_launch_description

    return generate_launch_description


class _BetterLaunchMeta(type):
    # Allows reusing an already existing BetterLaunch instance.
    # Important for launch file includes.
    def __call__(cls, *args, **kwargs):
        existing_instance = globals().get(__better_launch_instance, None)
        if existing_instance:
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

        if ns is None:
            ns = "/"
        elif not ns.startswith("/"):
            ns = "/" + ns

        self._group_stack = deque()
        self._group_stack.append(Group(self, ns))

        self._composition_node = None

    def _get_namespace(self, until: int = -1):
        ns = ""
        for g in self._group_stack[:until]:
            if g.ns:
                ns += "/" + g.ns.strip("/")

        return ns

    def _get_unique_name(self, name: str = ""):
        return name + "_" + __uuid_generator()

    def _on_sigint(self, signum):
        base_msg = "user interrupted with ctrl-c (SIGINT)"
        if not self.sigint_received:
            self.logger.warning(base_msg)
            self._shutdown(reason="ctrl-c (SIGINT)", was_sigint=True)
            self.sigint_received = True
        else:
            self.logger.warning("{} again, ignoring...".format(base_msg))

    def _on_sigterm(self, signum):
        signame = signal.Signals(signum).name
        self.logger.error("user interrupted with ctrl-\\ ({}), terminating...".format(signame))
        
        try:
            # Python 3.7+
            current_task = asyncio.current_task(self.asyncio_loop)
        except AttributeError:
            current_task = asyncio.Task.current_task(self.asyncio_loop)
        
        self.asyncio_loop.call_soon(current_task.cancel)

    def _shutdown(reason: str, was_sigint: bool = False):
        # TODO tell all nodes to shut down
        pass

    def find(self, pkg: str, filename: str = None, dir: str = None):
        # TODO
        pass

    @contextmanager
    def group(self, ns: str = None):
        group = Group(self, self._group_stack[-1], ns)
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
        # TODO pass namespace to node

        if anonymous:
            name = self._get_unique_name(name)

        if hidden and not name.startswith("_"):
            name = "_" + name

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

        self._group_stack[-1].add_node(node)
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

        # TODO pass namespace to node

        if anonymous:
            name = self._get_unique_name(name)

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
            self._group_stack[-1].add_node(comp)
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
            name = self._get_unique_name(name)

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
                        # TODO mechanism for passing instance not fully defined yet
                        global_args["_better_launcher_instance"] = self
                        exec(code, global_args, local_args)
                        return
                    except Exception as e:
                        # TODO log
                        raise e
                else:
                    # Assume a normal ros2 launch file
                    # TODO
                    pass

    # TODO Common stuff like joint_state_publisher, robot_state_publisher, moveit, rviz, gazebo
