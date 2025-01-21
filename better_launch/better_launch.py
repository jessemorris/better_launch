from typing import Any, Callable
import os
import platform
from ast import literal_eval
import signal
import inspect
import threading
from concurrent.futures import Future
from contextlib import contextmanager
from collections import deque
import logging
import yaml

from ament_index_python.packages import get_package_prefix

try:
    # For anonymous nodes
    import wonderwords

    __uuid_generator = lambda g=wonderwords.RandomWord(
        exclude_with_spaces=True
    ): g.word()
except ImportError:
    import uuid

    __uuid_generator = lambda: uuid.uuid4().hex

from elements import Group, Node, Composer, LifecycleNode, LifecycleStage
from ros.ros_adapter import ROSAdapter
from ros import logging as roslog
from utils.log_formatter import RosLogFormatter, default_colormap
from utils.substitutions import default_substitution_handlers, substitute_tokens


# TODO can we detect if we were called from the cli via "ros2 launch"?


_is_launcher_defined = "__better_launch_this_defined"
_has_bl_instance = "__better_launch_instance"


def launch_this(launch_func):
    glob = globals()
    if _is_launcher_defined in glob and _has_bl_instance not in glob:
        # Allow using launch_this only once unless we got included from another file
        raise RuntimeError("Can only use one launch decorator")

    glob[_is_launcher_defined] = True

    # Expose launch_func args through click?
    import click

    options = []
    sig = inspect.signature(launch_func)
    for param in sig.parameters.values():
        default = param.default
        options.append(click.Option(f"--{param.name}", default=default))

    def run():
        launch_func()

        # Retrieve the BetterLaunch singleton and run it
        BetterLaunch().spin()

    # All BL nodes and includes are async once started
    click_cmd = click.Command("main", callback=run, params=options)
    try:
        click_cmd.main()
    except SystemExit as e:
        if e.code != 0:
            raise


def ros2_opaque_function(launch_func, make_global: bool = True):
    # Makes your main function compatible with the ros2 launch system, e.g.
    # declares arguments, creates a stub launch description, etc.
    glob = globals()
    if _is_launcher_defined in glob and _has_bl_instance not in glob:
        # Allow using launch_this only once unless we got included from another file
        raise RuntimeError("Can only use one launch decorator")

    glob[_is_launcher_defined] = True

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

            # TODO spin BetterLaunch instance
            launch_func(**launch_args)

            # Retrieve the BetterLaunch singleton
            bl = BetterLaunch()
            # Opaque functions are allowed to return additional actions
            return bl._deferred_ros_actions

        ld.add_action(OpaqueFunction(ros2_wrapper))
        return ld

    if make_global:
        glob["generate_launch_description"] = generate_launch_description

    return generate_launch_description


class _BetterLaunchMeta(type):
    # Allows reusing an already existing BetterLaunch instance.
    # Important for launch file includes.
    def __call__(cls, *args, **kwargs):
        existing_instance = globals().get(_has_bl_instance, None)
        if existing_instance is not None:
            return existing_instance

        obj = cls.__new__(cls, *args, **kwargs)
        globals()[_has_bl_instance] = obj
        obj.__init__(*args, **kwargs)
        return obj


class BetterLaunch(metaclass=_BetterLaunchMeta):
    def __init__(
        self,
        ns: str = "/",
        launch_args: dict = None,
        name: str = None,
        *,
        log_level: int = logging.INFO,
    ):
        if launch_args is None:
            frame = inspect.currentframe()
            try:
                # Get the caller's locals
                launch_args = {
                    k: v
                    for k, v in frame.f_back.f_locals.items()
                    if not k.startswith("_")
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

        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigterm)

        if platform.system() != "Windows":
            signal.signal(signal.SIGQUIT, self._on_sigterm)

        # For those cases where we need to interact with ROS somehow (e.g. service calls)
        self.ros_adapter = ROSAdapter()
        self._shutdown_future = Future()

        self._logscreen_handler = None
        self._logfile_handlers = {}

        roslog.launch_config.level = log_level
        if "OVERRIDE_LAUNCH_SCREEN_FORMAT" not in os.environ:
            colormap = dict(default_colormap)
            colormap[logging.INFO] = "\x1b[32;20m"
            roslog.launch_config.screen_formatter = RosLogFormatter(colormap=colormap)

        self.logger = roslog.get_logger(name)

        self.all_args = launch_args
        self.sigint_received = False

        if ns is None:
            ns = "/"
        elif not ns.startswith("/"):
            ns = "/" + ns

        self._group_root = Group(self, None, ns)
        self._group_stack = deque()
        self._group_stack.append(self._group_root)

        self._composition_node = None

        # Allows to run traditional ros2 launch descriptions
        self._ros2_launcher = None
        self._ros2_launcher_thread = None

        self.hello()

    def hello(self):
        self.logger.info(
            # Ascii art: https://asciiart.cc/view/10677
            f"""\
Better Launch is starting!
Please fasten your seatbelts and secure all baggage underneath your chair.

Default log level is \x1b[34;20m{roslog.launch_config.level}\x1b[0m
All log files can be found under \x1b[34;20m{roslog.launch_config.log_dir}\x1b[0m

Takeoff in 3... 2... 1...

                        ,:
                      ,' |
                     /   :
                  --'   /
                  \/ /:/
                  / ://_\\
               __/   /
               )'-. /
               ./  :\\
                /.' '
              '/'
              +
           .-"-
          (    )
       . .-'  '.
      ( (.   )8:
  .'    / (_  )
"""
        )

    def spin(self):
        if self._ros2_launcher and not self._ros2_launcher_thread:
            # Apply our config to the ROS2 launch logging config
            import launch

            launch.logging.launch_config = roslog.launch_config

            self._ros2_launcher_thread = threading.Thread(
                target=self._ros2_launcher.run,
                daemon=True,
            )
            self._ros2_launcher_thread.start()

        self.ros_adapter._thread.join()

    def get_unique_name(self, name: str = ""):
        return name + "_" + __uuid_generator()

    def all_groups(self):
        # Assemble all groups
        groups: list[Group] = [self.group_root]
        queue: list[Group] = [self.group_root]

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

    @property
    def group_root(self) -> Group:
        return self._group_stack[0]

    @property
    def group_tip(self) -> Group:
        return self._group_stack[-1]

    def _on_sigint(self, sig, frame):
        if not self.sigint_received:
            self.logger.warning(f"Received (SIGINT), forwarding to child processes...")
            self.shutdown("user interrupt", signal.SIGINT)
            self.sigint_received = True
        else:
            self.logger.warning(f"Received (SIGINT) again, escalating to sigterm")
            self._on_sigterm(sig, frame)

    def _on_sigterm(self, sig, frame):
        self.logger.error(f"Using (SIGTERM) can result in orphaned processes!")

        # Final chance for the processes to shut down, but we will no longer wait
        self.shutdown(f"received (SIGTERM)", signal.SIGTERM)

        if not self.is_shutdown:
            self._shutdown_future.cancel()

    @property
    def is_shutdown(self):
        return self._shutdown_future.done()

    def shutdown(self, reason: str, signum: int = signal.SIGTERM):
        self.ros_adapter.shutdown()

        # Tell all nodes to shut down
        for n in self.all_nodes():
            n.shutdown(reason, signum)

        # If we launched extra ROS2 actions tell the launch service to shut down, too
        if self._ros2_launcher is not None:
            self._ros2_launcher.shutdown()

        try:
            self._shutdown_future.set_result(None)
        except:
            pass

    def find(self, package: str, file_name: str = None, file_dir: str = None):
        package_dir = get_package_prefix(package) if package else None

        if file_name is None:
            return package_dir

        if package_dir is None:
            return file_name

        # look in specific subolder
        if file_dir is not None:
            package_dir = os.path.join(package_dir, file_dir)

        # look for it
        for root, dirs, files in os.walk(package_dir, topdown=False):
            if file_name in files:
                return os.path.join(package_dir, root, file_name)

        # not there
        raise RuntimeError(f"Could not find file {file_name} in package {package}")

    def resolve_string(self, s: str) -> str:
        return substitute_tokens(s, default_substitution_handlers(self, "full"))

    def load_params(self, path: str, node_or_namespace: str | Node = None):
        path = self.resolve_string(path)

        with open(path) as f:
            params = yaml.safe_load(f)

        if node_or_namespace:
            ns = node_or_namespace
            if isinstance(ns, Node):
                ns = ns.fullname

            # Root namespace is typically not part of param files, but we may have to adjust this
            # Parameter files can use one or more elements from the namespace to define sections
            parts = ns.strip("/").split("/")
            idx = 0
            while idx < len(parts):
                key = parts[idx]

                while key not in params and idx < len(parts):
                    idx += 1
                    key += "/" + parts[idx]

                if key not in params:
                    raise ValueError(f"Could not find parameter section for {ns}")

                params = params[key]
                idx += 1

        if "ros__parameters" in params:
            params = params["ros__parameters"]

        return params

    @contextmanager
    def group(self, ns: str = None):
        if self._composition_node:
            raise RuntimeError("Cannot add groups inside a composition node")

        group = Group(self, self.group_tip, ns)
        self.group_tip.add_group(group)
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
        reparse_logs: bool = True,
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
        start_immediately: bool = True,
        **kwargs,
    ):
        if self._composition_node:
            raise RuntimeError("Cannot add nodes inside a composition node")

        if anonymous:
            name = self.get_unique_name(name)

        if hidden and not name.startswith("_"):
            name = "_" + name

        # Assemble additional node remaps from our group branch
        g = self.group_tip
        remaps = g.assemble_remaps()
        if remap:
            remaps.update(remap)

        # Why do I hear mad hatter music???
        # launch_ros/actions/node.py:497
        ns = g.assemble_namespace()
        remaps["__ns"] = ns

        exec_file = self.find(pkg, exec)
        node = Node(
            self,
            exec_file,
            name,
            node_args,
            reparse_logs=reparse_logs,
            remap=remaps,
            env=env,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
            emulate_tty=emulate_tty,
            stderr_to_stdout=stderr_to_stdout,
            start_immediately=start_immediately,
            **kwargs,
        )

        g.add_node(node)
        return node

    def lifecycle_node(
        self,
        pkg: str,
        exec: str,
        name: str = None,
        node_args: dict[str, Any] = None,
        target_state: LifecycleStage = LifecycleStage.PRISTINE,
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
        # TODO a lot of this is redundant with node() -> common function?
        if self._composition_node:
            raise RuntimeError("Cannot add nodes inside a composition node")

        if anonymous:
            name = self.get_unique_name(name)

        if hidden and not name.startswith("_"):
            name = "_" + name

        # Assemble additional node remaps from our group branch
        g = self.group_tip
        remaps = g.assemble_remaps()
        if remap:
            remaps.update(remap)

        # Why do I hear mad hatter music???
        # launch_ros/actions/node.py:497
        ns = g.assemble_namespace()
        remaps["__ns"] = ns

        exec_file = self.find(pkg, exec)
        node = LifecycleNode(
            self,
            exec_file,
            name,
            node_args,
            target_state,
            remap=remaps,
            env=env,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
            emulate_tty=emulate_tty,
            stderr_to_stdout=stderr_to_stdout,
            **kwargs,
        )

        g.add_node(node)
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

        # Remaps apply to the
        # Assemble additional node remaps from our group branch
        g = self.group_tip
        remaps = g.assemble_remaps()
        remaps.update(remap)

        # Why do I hear mad hatter music???
        # launch_ros/actions/node.py:497
        ns = g.assemble_namespace()
        remaps["__ns"] = ns

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
            start_immediately=True,  # always start, otherwise adding components is troublesome
            **kwargs,
        )

        try:
            g.add_node(comp)
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

        # TODO arguments
        self._composition_node.add_component(
            pkg,
            plugin,
            name,
            anonymous=anonymous,
            hidden=hidden,
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
                    # Assume launch file uses better_launch, too
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
                    self._include_ros2(file_path)
        else:
            self._include_ros2(file_path)

    def _include_ros2(self, file_path):
        # Delegate to ros2 launch service
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import (
            PythonLaunchDescriptionSource,
        )

        ros2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(file_path)
        )

        self.launchdescription(ros2_include)

    def launchdescription(self, launch_description):
        from launch import LaunchService

        if self._ros2_launcher is None:
            self._ros2_launcher = LaunchService(noninteractive=True)

        self._ros2_launcher.include_launch_description(launch_description)
