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

from rclpy.qos import QoSProfile
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
from utils.better_logging import (
    log_default_colormap,
    RosLogFormatter,
    LogRecordForwarder,
)
from utils.substitutions import default_substitution_handlers, substitute_tokens
from ros.ros_adapter import ROSAdapter
from ros import logging as roslog
from ros.logging import LaunchConfig as LogConfig


_is_launcher_defined = "__better_launch_this_defined"
_bl_singleton_instance = "__better_launch_instance"


def _expose_ros2_launch_function(launch_func):
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
                    launch_args[k] = v

            # Call the launch function
            launch_func(**launch_args)

            # Retrieve the BetterLaunch singleton
            bl = BetterLaunch()

            # Opaque functions are allowed to return additional actions
            return bl._ros2_actions

        ld.add_action(OpaqueFunction(ros2_wrapper))
        return ld

    # Add our generate_launch_description function to the module launch_this was called from
    stack = inspect.stack()
    for i, frame_info in enumerate(stack):
        if frame_info.function.startswith("launch_this"):
            if i + 1 >= len(stack):
                raise RuntimeError("Could not determine the calling module")
            launchfile_frame = stack[i + 1]
            caller_globals = launchfile_frame.frame.f_globals
            caller_globals["generate_launch_description"] = generate_launch_description
            break
    else:
        raise RuntimeError("This function must be called through launch_this")


def launch_this(
    launch_func,
    ui: bool = True,
    join: bool = True,
    log_config: LogConfig = None,
):
    glob = globals()
    if _is_launcher_defined in glob and _bl_singleton_instance not in glob:
        # Allow using launch_this only once unless we got included from another file
        raise RuntimeError("Can only use one launch decorator")

    glob[_is_launcher_defined] = True

    # Get the filename of the original launchfile
    # NOTE be careful not to instantiate BetterLaunch before launch_func has run
    if not _bl_singleton_instance in glob:
        stack = inspect.stack()
        for frame_info in stack:
            if frame_info.function.startswith("launch_this"):
                BetterLaunch.launchfile = frame_info.filename
                break
        else:
            print("Launchfile resolution failed")

        del frame_info

    # Signal handlers have to be installed on the main thread. Since the BetterLaunch singleton
    # could be instantiated first on a different thread we do it here where we can set stronger
    # restrictions.
    def sigint_handler(sig, frame):
        BetterLaunch()._on_sigint(sig, frame)

    def sigterm_handler(sig, frame):
        BetterLaunch()._on_sigint(sig, frame)

    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigterm_handler)

    if platform.system() != "Windows":
        signal.signal(signal.SIGQUIT, sigterm_handler)

    # If we were started by ros launch (e.g. through 'ros2 launch <some-bl-launch-file>') we need
    # to expose a "generate_launch_description" method instead of running by ourselves.
    #
    # Launch files in ROS2 are run by adding an IncludeLaunchDescription action to the
    # LaunchService (both found in https://github.com/ros2/launch/). When the action is resolved,
    # it ultimately leads to get_launch_description_from_python_launch_file, which imports the file
    # and then checks for a generate_launch_description function.
    #
    # See the following links for details:
    #
    # https://github.com/ros2/launch_ros/blob/rolling/ros2launch/ros2launch/command/launch.py#L125
    # https://github.com/ros2/launch_ros/blob/rolling/ros2launch/ros2launch/api/api.py#L141
    # https://github.com/ros2/launch/blob/rolling/launch/launch/actions/include_launch_description.py#L148
    # https://github.com/ros2/launch/blob/rolling/launch/launch/launch_description_sources/python_launch_file_utilities.py#L43
    stack = inspect.stack()
    for frame_info in stack:
        frame_locals = frame_info.frame.f_locals
        if "self" not in frame_locals:
            continue
        owner = frame_locals["self"]
        if isinstance(owner, object) and owner.__name__ == "IncludeLaunchDescription":
            # We were included, expose the expected method in our caller's globals and return
            _expose_ros2_launch_function(launch_func)
            return

    # If we get here we were not included by ROS2

    # Disable UI when included from another launch file
    if _bl_singleton_instance in glob:
        ui = False

    # Logging setup
    if log_config:
        roslog.launch_config = log_config
        # roslog.reset()
    else:
        roslog.launch_config.level = logging.INFO
        if "OVERRIDE_LAUNCH_SCREEN_FORMAT" not in os.environ:
            colormap = dict(log_default_colormap)
            colormap[logging.INFO] = "\x1b[32;20m"
            roslog.launch_config.screen_formatter = RosLogFormatter(colormap=colormap)

    # Expose launch_func args through click. This enables using launch files like other
    # python files, e.g. './my_better_launchfile.py --help'
    import click

    options = []
    sig = inspect.signature(launch_func)

    # Optional: extract more fine-grained information from the docstring
    try:
        from docstring_parser import parse as parse_docstring

        _doc = parse_docstring(launch_func.__doc__)
        launch_func_doc = _doc.short_description
        param_docstrings = {p.arg_name: p.description for p in _doc.params}
    except ImportError:
        launch_func_doc = launch_func.__doc__
        param_docstrings = {}

    # Create CLI options for click
    for param in sig.parameters.values():
        default = None
        if param.default is not param.empty:
            default = param.default

        ptype = None
        if default is None and param.annotation is not param.empty:
            ptype = param.annotation

        options.append(
            click.Option(
                [f"--{param.name}"],
                type=ptype,
                default=default,
                help=param_docstrings.get(param, param),
            )
        )

    # Wrap the launch function so we can do some preparation and cleanup tasks
    def launch_func_wrapper(*args, **kwargs):
        launch_func(*args, **kwargs)

        # Retrieve the BetterLaunch singleton
        bl = BetterLaunch()
        bl.execute_pending_ros_actions(join=join and not ui)

    def run(*args, **kwargs):
        if ui:
            from tui.pyterm_app import BetterUI

            BetterUI.setup_logging()
            app = BetterUI()
            app.start(launch_func_wrapper, *args, **kwargs)
        else:
            launch_func_wrapper(*args, **kwargs)

    click_cmd = click.Command(
        "main", callback=run, params=options, help=launch_func_doc
    )
    try:
        click_cmd.main()
    except SystemExit as e:
        if e.code != 0:
            raise


class _BetterLaunchMeta(type):
    _singleton_future = Future()

    # Allows reusing an already existing BetterLaunch instance.
    # Important for launch file includes.
    # TODO should calling again with different args have any effect on the existing instance?
    def __call__(cls, *args, **kwargs):
        existing_instance = globals().get(_bl_singleton_instance, None)
        if existing_instance is not None:
            return existing_instance

        obj = cls.__new__(cls, *args, **kwargs)
        globals()[_bl_singleton_instance] = obj
        obj.__init__(*args, **kwargs)

        cls._singleton_future.set_result(obj)
        return obj

    def wait_for_instance(cls, timeout: float = None) -> "BetterLaunch":
        return cls._singleton_future.result(timeout)


class BetterLaunch(metaclass=_BetterLaunchMeta):
    launchfile: str = None

    def __init__(
        self,
        name: str = None,
        launch_args: dict = None,
        root_namespace: str = "/",
    ):
        if not name:
            name = os.path.basename(BetterLaunch.launchfile)
            name = os.path.splitext(name)[0]
            if name.endswith(".launch"):
                name = os.path.splitext(name)[0]

        # roslog.launch_config must be setup before instantiation of BetterLaunch
        self.logger = roslog.get_logger(name)

        if launch_args is None:
            # Retrieve the arguments of the function that launch_this was decorating
            stack = inspect.stack()
            for i, frame_info in enumerate(stack):
                if frame_info.function == "launch_this":
                    if i + 1 >= len(stack):
                        raise RuntimeError("Could not determine the launch function")
                    launch_frame = stack[i - 1]
                    calling_function = launch_frame.frame.f_globals[
                        launch_frame.function
                    ]
                    sig = inspect.signature(calling_function)
                    bound_args = sig.bind(**launch_frame.frame.f_locals)
                    bound_args.apply_defaults()
                    launch_args = dict(bound_args.arguments)
                    break

            del frame_info

        self.launch_args = launch_args

        # For those cases where we need to interact with ROS (e.g. service calls)
        self.ros_adapter = ROSAdapter()

        if root_namespace is None:
            root_namespace = "/"
        root_namespace = "/" + root_namespace.strip("/")

        self._group_root = Group(self, None, root_namespace)
        self._group_stack = deque()
        self._group_stack.append(self._group_root)

        self._composition_node = None

        # Allows to run traditional ros2 launch descriptions
        self._ros2_actions = []
        self._ros2_launcher = None
        self._ros2_launcher_thread = None

        self._sigint_received = False
        self._shutdown_future = Future()
        self._shutdown_callbacks = []

        self.hello()

    @staticmethod
    def ros_version():
        """
        Returns the name of the currently sourced ros version (e.g. $ROS_VERSION)
        """
        return os.environ["ROS_DISTRO"]

    def hello(self):
        self.logger.info(
            # Ascii art based on: https://asciiart.cc/view/10677
            f"""
\x1b[1;20mBetter Launch is starting!\x1b[0m
Please fasten your seatbelts and secure all baggage underneath your chair.

Default log level is \x1b[34;20m{roslog.launch_config.level} ({logging.getLevelName(roslog.launch_config.level)})\x1b[0m
All log files can be found under \x1b[34;20m{roslog.launch_config.log_dir}\x1b[0m

Takeoff in 3... 2... 1...

           *            ,:
    +                 ,' |
               +     /   :
       *          --'   /
+                 \/ /:/
            *     / ://_\\
       +       __/   /
  -            )'-. /
               ./  :\\
        *       /.' '
              '/'
   '          +
           .-"-
          (    )
       . .-'  '.
      ( (.   )8:
  .' _  / (_  ) '._
"""
        )

    def execute_pending_ros_actions(self, join: bool = True):
        if self._ros2_actions:
            # Apply our config to the ROS2 launch logging config
            import launch

            launch.logging.launch_config = roslog.launch_config

            self.logger.info("Forwarding pending ROS2 actions to launch service")
            if not self._ros2_launcher:
                if self._ros2_launcher is None:
                    self._ros2_launcher = launch.LaunchService(noninteractive=True)

            ld = launch.LaunchDescription(self._ros2_actions)
            self._ros2_actions.clear()
            self._ros2_launcher.include_launch_description(ld)

        if self._ros2_launcher:
            if not (
                self._ros2_launcher_thread and self._ros2_launcher_thread.is_alive()
            ):
                self.logger.info("Starting ROS2 launch service")
                self._ros2_launcher_thread = threading.Thread(
                    target=self._ros2_launcher.run,
                    daemon=True,
                )
                self._ros2_launcher_thread.start()

            if join:
                self.spin()
        else:
            self.logger.info("No ROS2 actions pending")

    def spin(self):
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
    def shared_node(self):
        return self.ros_adapter.ros_node

    @property
    def group_root(self) -> Group:
        return self._group_stack[0]

    @property
    def group_tip(self) -> Group:
        return self._group_stack[-1]

    def _on_sigint(self, sig, frame):
        if not self._sigint_received:
            self.logger.warning(f"Received (SIGINT), forwarding to child processes...")
            self.shutdown("user interrupt", signal.SIGINT)
            self._sigint_received = True
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

    def add_shutdown_callback(self, callback: Callable):
        self._shutdown_callbacks.append(callback)

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

        # Call any callbacks, but only once
        callbacks = self._shutdown_callbacks
        self._shutdown_callbacks = []

        for cb in callbacks:
            try:
                cb()
            except Exception as e:
                self.logger.warning(f"Shutdown callback failed: {e}")

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

    def service(
        self,
        topic: str,
        service_type: type,
        callback: Callable,
        qos_profile: QoSProfile | int = 10,
    ):
        return self.ros_node.create_service(
            service_type,
            topic,
            callback,
            qos_profile=qos_profile,
        )

    def service_client(
        self,
        topic: str,
        service_type: type,
        timeout: float = 0.0,
        qos_profile: QoSProfile | int = 10,
    ):
        client = self.ros_node.create_client(
            service_type, topic, qos_profile=qos_profile
        )
        if timeout > 0.0:
            if not client.wait_for_service(timeout):
                raise ValueError(f"Service client timed out ({topic}, {service_type})")
        return client

    def publisher(
        self, topic: str, message_type: type, qos_profile: QoSProfile | int = 10
    ):
        return self.ros_node.create_publisher(
            message_type,
            topic,
            qos_profile=qos_profile,
        )

    def subscriber(
        self,
        topic: str,
        message_type: type,
        callback: Callable,
        qos_profile: QoSProfile | int = 10,
    ):
        return self.ros_node.create_subscriber(
            message_type,
            topic,
            callback,
            qos_profile=qos_profile,
        )

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
        log_level: int = logging.INFO,
        output_config: str | dict[str, set[str]] = "screen",
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
            log_level=log_level,
            output_config=output_config,
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
        log_level: int = logging.INFO,
        output_config: str | dict[str, set[str]] = "screen",
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
            log_level=log_level,
            output_config=output_config,
            reparse_logs=reparse_logs,
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
        log_level: int = logging.INFO,
        output_config: str | dict[str, set[str]] = "screen",
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
            process_log_level=log_level,
            output_config=output_config,
            reparse_logs=reparse_logs,
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
        if pass_all_args:
            local_args = self.launch_args
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
                        global_args = dict(globals())
                        global_args["_better_launcher_instance"] = self
                        exec(code, global_args, local_args)
                        return
                    except Exception as e:
                        self.logger.error(
                            f"Launch include '{pkg}/{launch_file}' failed: {e}"
                        )
                        raise

        # TODO could be stricter about verification here
        # Was not a better_launch launch file, assume it's a ROS2 launch file (py, xml, yaml)
        self._make_ros2_include(file_path, **local_args)

    def _make_ros2_include(self, file_path, **kwargs):
        # Delegate to ros2 launch service
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import (
            AnyLaunchDescriptionSource,
        )

        # See https://github.com/ros2/launch_ros/blob/rolling/ros2launch/ros2launch/api/api.py#L175
        ros2_include = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(file_path),
            launch_arguments=[(key, val) for key, val in kwargs.items()],
        )
        self.ros2_action(ros2_include)

    def ros2_action(self, ros2_action):
        self._ros2_actions.append(ros2_action)
