from typing import Any, Callable, Generator, overload, TYPE_CHECKING
import importlib
import sys
import os
import signal
import inspect
import time
import regex
import threading
from concurrent.futures import Future
from contextlib import contextmanager
from collections import deque
import logging
import yaml

from rclpy.node import (
    Node as RosNode,
    Service as RosServiceProvider,
    Client as RosServiceClient,
    Publisher as RosPublisher,
    Subscription as RosSubscriber,
)
from rclpy.qos import QoSProfile, qos_profile_services_default
from ament_index_python.packages import get_package_prefix

if TYPE_CHECKING:
    # Surprisingly large imports, so we only import them if we actually need them
    from rclpy.action import (
        ActionServer as RosActionServer,
        ActionClient as RosActionClient,
    )

try:
    # For anonymous nodes
    import wonderwords

    __uuid_generator = lambda g=wonderwords.RandomWord(
        exclude_with_spaces=True
    ): g.word()
except ImportError:
    import uuid

    __uuid_generator = lambda: uuid.uuid4().hex

from better_launch.elements import (
    Group,
    AbstractNode,
    Node,
    Composer,
    Component,
    LifecycleStage,
    Ros2LaunchWrapper,
)
from better_launch.utils.better_logging import default_log_colormap, PrettyLogFormatter
from better_launch.utils.substitutions import (
    default_substitution_handlers,
    substitute_tokens,
)
from better_launch.utils.introspection import (
    find_function_frame,
    find_calling_frame,
    find_launchthis_function,
)
from better_launch.ros.ros_adapter import ROSAdapter
from better_launch.ros import logging as roslog
from better_launch.ros.logging import LaunchConfig as LogConfig


_bl_singleton_instance = "__better_launch_instance"
_bl_include_args = "__better_launch_include_args"


class _BetterLaunchMeta(type):
    _singleton_future = Future()

    # Allows reusing an already existing BetterLaunch instance.
    # Important for launch file includes.
    def __call__(cls, *args, **kwargs):
        existing_instance = globals().get(_bl_singleton_instance, None)
        if existing_instance is not None:
            # TODO raise or warn?
            return existing_instance

        obj = cls.__new__(cls, *args, **kwargs)
        globals()[_bl_singleton_instance] = obj
        obj.__init__(*args, **kwargs)

        cls._singleton_future.set_result(obj)
        return obj

    def instance(cls) -> "BetterLaunch":
        try:
            return cls._singleton_future.result(0.0)
        except TimeoutError:
            return None

    def wait_for_instance(cls, timeout: float = None) -> "BetterLaunch":
        return cls._singleton_future.result(timeout)


class BetterLaunch(metaclass=_BetterLaunchMeta):
    """This should be all you need to create beautiful, simple and convenient launch files!"""

    _launchfile: str = None
    _launch_func_args: dict[str, Any] = {}

    def __init__(
        self,
        name: str = None,
        launch_args: dict[str, Any] = None,
        root_namespace: str = "/",
    ):
        """Note that BetterLaunch is a singleton: only the first invocation to `__init__` will succeed. All subsequent calls will return the previous instance. If you need access to the BetterLaunch instance outside your launch function, consider using one of the following classmethods instead:
        * :py:meth:`BetterLaunch.instance <_BetterLaunchMeta.instance>`
        * :py:meth:`BetterLaunch.wait_for_instance <_BetterLaunchMeta.wait_for_instance>`

        Parameters
        ----------
        name : str, optional
            The name of this instance, will default to the launchfile's filename.
        launch_args : dict, optional
            Override the launch arguments BetterLaunch has access to. By default this will be the launch function's arguments. These will mainly be used for passing to included launch files.
        root_namespace : str, optional
            The namespace of the root group.
        """
        if not name:
            if not BetterLaunch._launch_func_args:
                frame = find_calling_frame(self.__init__)
                BetterLaunch._launchfile = frame.filename
            name = os.path.basename(BetterLaunch._launchfile)

        # roslog.launch_config must be setup before instantiation of BetterLaunch
        self.logger = roslog.get_logger(name)

        if launch_args is not None:
            BetterLaunch._launch_func_args = launch_args

        # For those cases where we need to interact with ROS (e.g. service calls)
        self.ros_adapter = ROSAdapter()

        if root_namespace is None:
            root_namespace = "/"
        root_namespace = "/" + root_namespace.strip("/")

        self._group_root = Group(None, root_namespace)
        self._group_stack = deque()
        self._group_stack.append(self._group_root)

        self._composition_node = None

        # Allows to run traditional ros2 launch actions and descriptions
        self._ros2_launcher = None

        self._sigint_received = False
        self._sigterm_received = False
        self._shutdown_future = Future()
        self._shutdown_callbacks = []

        self.hello()

    def hello(self) -> None:
        """Prints our welcome message and some useful information.
        Note that this will not appear in the logs!
        """
        # Ascii art based on: https://asciiart.cc/view/10677
        msg = f"""
\x1b[1;20mBetter Launch is starting!\x1b[0m
Please fasten your seatbelts and secure all baggage underneath your chair.

Default log level is \x1b[34;20m{roslog.launch_config.level} ({logging.getLevelName(roslog.launch_config.level)})\x1b[0m
All log files can be found at
\x1b[34;20m{roslog.launch_config.log_dir}\x1b[0m

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
        # We don't want to log this
        print(msg)
        self.logger.critical(f"Log files at {roslog.launch_config.log_dir}")

    def spin(self) -> None:
        """Join the BetterLaunch thread until it terminates."""
        self.ros_adapter._thread.join()

    def get_unique_name(self, name: str = "") -> str:
        """Adds a unique suffix to the provided string.

        Parameters
        ----------
        name : str, optional
            The string to use as the base.

        Returns
        -------
        str
            The passed in string with a unique suffix.
        """
        return name + "_" + __uuid_generator()

    def all_groups(self) -> list[Group]:
        """Returns a list of all in the order they were created.

        Returns
        -------
        list[Group]
            All groups added so far.
        """
        # Assemble all groups
        groups: list[Group] = [self.group_root]
        queue: list[Group] = [self.group_root]

        # Simplified breadth first search since we don't expect any loops
        while queue:
            g = queue.pop()
            groups.extend(g.children)
            queue.extend(g.children)

        return groups

    def all_nodes(
        self, include_components: bool = False, include_launch_service: bool = True
    ) -> list[AbstractNode]:
        """Returns a list of all nodes in the order they were added. Components will be added right after their composers. If a ROS2 launch service has been started it will be added at the very end.

        Parameters
        ----------
        include_components : bool, optional
            Whether to include :py:class:`Component` instances.

        include_launch_service : bool, optional
            Whether to include the ROS2 launch service wrapper if it was created.

        Returns
        -------
        list[AbstractNode]
            A list of all nodes, sorted by when they were added.
        """
        nodes = []
        groups = self.all_groups()

        for g in groups:
            for n in g.nodes:
                nodes.append(n)
                if include_components and isinstance(n, Composer):
                    nodes.extend(n.loaded_components)

        if include_launch_service and self._ros2_launcher:
            nodes.append(self._ros2_launcher)

        return nodes

    def query_node(
        self,
        name_regex: str,
        *,
        include_components: bool = True,
        include_launch_service: bool = False,
    ) -> AbstractNode:
        """Retrieve the first node whos :py:meth:`AbstractNode.fullname` matches the provided regex.

        Parameters
        ----------
        name_regex : str
            The regex to match the nodes' full names against.
        include_components : bool, optional
            Whether to include components in the results, if any.
        include_launch_service : bool, optional
            Whether to include the ROS2 launch service in the result (if it exists).

        Returns
        -------
        AbstractNode
            The first node matching the provided regex, or None if none matched.
        """
        reg = regex.compile(name_regex)
        for node in self.all_nodes(
            include_components=include_components,
            include_launch_service=include_launch_service,
        ):
            if reg.match(node.fullname):
                return node
        
        return None

    def query_nodes(
        self,
        name_regex: str,
        *,
        include_components: bool = True,
        include_launch_service: bool = False,
    ) -> list[AbstractNode]:
        """Retrieve all nodes whos :py:meth:`AbstractNode.fullname` matches the provided regex.

        Parameters
        ----------
        name_regex : str
            The regex to match the nodes' full names against.
        include_components : bool, optional
            Whether to include components in the results, if any.
        include_launch_service : bool, optional
            Whether to include the ROS2 launch service in the result (if it exists).

        Returns
        -------
        list[AbstractNode]
            A list of all nodes matching the regex.
        """
        reg = regex.compile(name_regex)
        return [
            node 
            for node in self.all_nodes(
                include_components=include_components,
                include_launch_service=include_launch_service,
            )
            if reg.match(node.fullname)
        ]

    @staticmethod
    def ros_version() -> str:
        """Returns the name of the currently sourced ros version (i.e. *$ROS_VERSION*)."""
        return os.environ["ROS_DISTRO"]

    @property
    def launchfile(self) -> str:
        """The path of the (main) *better_launch* launchfile being executed."""
        return BetterLaunch._launchfile

    @property
    def launch_args(self) -> dict[str, Any]:
        """All key-value pairs that have been passed to the launch function."""
        return BetterLaunch._launch_func_args

    @property
    def shared_node(self) -> RosNode:
        """A ROS2 node instance that can be used for creating publishers, services, etc."""
        return self.ros_adapter.ros_node

    @property
    def group_root(self) -> Group:
        """The root group ("/")."""
        return self._group_stack[0]

    @property
    def group_tip(self) -> Group:
        """The most recent group."""
        return self._group_stack[-1]

    def _on_sigint(self, sig: int, frame: inspect.FrameInfo) -> None:
        if not self._sigint_received:
            self.logger.warning(f"Received (SIGINT), forwarding to child processes...")
            self._sigint_received = True
            self.shutdown("user interrupt", signal.SIGINT)
        else:
            self.logger.warning(f"Received (SIGINT) again, escalating to sigterm")
            self._on_sigterm(sig, frame)

    def _on_sigterm(self, sig: int, frame: inspect.FrameInfo) -> None:
        if self._sigterm_received:
            try:
                self.logger.critical("(SIGTERM) received again, terminating process")
            finally:
                sys.exit(-1)

        self._sigterm_received = True
        self.logger.error(f"Using (SIGTERM) can result in orphaned processes!")

        # Final chance for the processes to shut down, but we will no longer wait
        self.shutdown(f"received (SIGTERM)", signal.SIGTERM)

        if not self.is_shutdown:
            self._shutdown_future.cancel()

    @property
    def is_shutdown(self) -> bool:
        """Whether *better_launch* has shutdown."""
        # TODO this is a remnant from the very beginning when we still took inspiration from ROS2 launch. Remove?
        return self._shutdown_future.done()

    def add_shutdown_callback(self, callback: Callable[[], Any]) -> None:
        """Adds a callback which will be called when *better_launch* shuts down.

        Parameters
        ----------
        callback : Callable
            The callback to call on shutdown.
        """
        self._shutdown_callbacks.append(callback)

    def shutdown(self, reason: str = None, signum: int = signal.SIGTERM) -> None:
        """Ask all nodes to shutdown and terminate the internal ROS2 thread. Any subsequent calls to BetterLaunch member functions, including this one, may fail. This will typically be called when you want to terminate your launch file.

        Parameters
        ----------
        reason : str, optional
            A human-readable string explaining the reason for the shutdown. If not given this will be requitted with a warning.
        signum : int, optional
            The signal to send to child processes.
        """
        if reason is None:
            try:
                frame = find_function_frame(self.shutdown)
                self.logger.warning(f"Shutdown was called from {frame.function}, but no reason was given")
            except:
                self.logger.warning(f"Shutdown was called without providing a reason and the calling frame could not be determined")

        # Tell all nodes to shut down
        for n in self.all_nodes():
            try:
                n.shutdown(reason, signum)
            except Exception as e:
                self.logger.error(
                    f"Node {n.name} raised an exception during shutdown: {e}"
                )

        try:
            self.ros_adapter.shutdown()
        except Exception as e:
            self.logger.error(f"RosAdapter raised an exception during shutdown: {e}")

        # If we launched extra ROS2 actions tell the launch service to shut down, too
        if self._ros2_launcher is not None:
            try:
                self._ros2_launcher.shutdown(reason, signum)
            except Exception as e:
                self.logger.error(
                    f"ROS2 launch service raised an exception during shutdown: {e}"
                )

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

    def find(
        self,
        *,
        filename: str = None,
        package: str = None,
        subdir: str = None,
        resolve_result: bool = True,
    ) -> str:
        """Resolve a path to a file or package.

        If the `filename` is absolute, all other arguments will be ignored and the filename will be returned.

        If `package` is provided, the corresponding ROS2 package path will be used as the base path. Else we attempt to locate the current launch file's package by searching its directory and parent directories for a `package.xml`. If the package cannot be determined the current working dir is used as the base path.

        If neither `subdir` nor `filename` are provided, the base path is returned. Otherwise, subdir and filename are concatenated to form a target path. The base path is then searched for any directory or file matching the target path.

        More specifically, if only `filename` is provided, a file with that name is located. If only `subdir` is provided, a matching path within the base path is located. If both `filename` and `subdir` are provided, a file within the specified path fragment is located.

        Parameters
        ----------
        filename : str, optional
            Name of a file to look for.
        package : str, optional
            Name of a ROS2 package to resolve.
        subdir : str, optional
            Path to add to the base path.
        resolve_result : bool, optional
            If True, the result will be passed through :py:metho:`resolve_string` before returning.

        Returns
        -------
        str
            A resolved path.

        Raises
        ------
        ValueError
            If `package` contains path separators, or if a `filename` is provided but could not be found within base path.
        """
        if resolve_result:
            resolve = self.resolve_string
        else:
            resolve = lambda s: s

        if filename:
            filename = resolve(filename)
            if os.path.isabs(filename):
                return filename

        if not package:
            # Search the launch file directory tree for a "package.xml"
            searchpath = os.path.normpath(os.path.dirname(self.launchfile))
            while os.pathsep in searchpath:
                files = os.listdir(searchpath)
                if "package.xml" in files:
                    package = os.path.basename(searchpath)
                    break
                searchpath = searchpath.rsplit(os.pathsep, maxsplit=1)[0]

        if package:
            if "/" in package or os.pathsep in package:
                raise ValueError("Package must be a single name, not a path")
            base_path = get_package_prefix(package)
        else:
            base_path = os.getcwd()

        base_path = resolve(base_path)
        if not filename and not subdir:
            return base_path

        targetpath = os.path.normpath(subdir) if subdir else ""
        for dirpath, _, files in os.walk(base_path, topdown=False):
            path = os.path.normpath(os.path.join(base_path, dirpath))
            if path.endswith(targetpath) and (not filename or filename in files):
                return os.path.join(path, filename)

        raise ValueError(
            f"Could not find file or directory (filename={filename}, package={package}, subdir={subdir})"
        )

    def resolve_string(self, s: str) -> str:
        """Replaces a variety of special strings in the provided string, usually a path.

        This is similar to what ROS1 could do when resolving paths in XML launch files. Substitutions always have the form `$(<substitution-type> <substitution-args>)`. Substitutions can also be nested, so the following is possible:

        ``$(eval $(arg x) * 5)``  ->  if x=2, this will be resolved to 10

        Note that the underlying algorithm will likely fail if it encounters additional brackets within the string.

        The following substitutions are supported:
        * `$(find <filename> <package> <subdir>)`: return the result of :py:meth:`find`. Note that substitution arguments are always sequential (not kwargs).
        * `$(arg <name> <default>)`: return the value of an argument passed to the launch function or `<default>` if it doesn't exist. Raises KeyError if no default is provided and no default was provided.
        * `$(param <full-node-name> <param>)`: retrieves the value of the ROS parameter `<param>` from the `<full-node-name>` (i.e. namespace + node name). Raises KeyError if the node does not exist or ValueError if the node does not have the specified parameter.
        * `$(env <key> <default>)`: return the value of the environment variable `<key>` or `<default>` if it doesn't exist. Raises KeyError if no default is provided and no default was provided.
        * `$(eval <python-snippet>)`: returns the result from evaluating the provided `<python-snippet>`. Typical use cases include simple math and assembling strings. **Note** that this indeed uses python's :py:func:`eval`.

        Parameters
        ----------
        s : str
            The string to apply substitutions to.

        Returns
        -------
        str
            The string with all substitutions involved.
        Raises
        ------
        KeyError, ValueError
            Depending on the substitution that failed.
        """
        if not s:
            return ""
        # TODO works but could be designed nicer. Consider these non-public API for now
        return substitute_tokens(s, default_substitution_handlers("full"))

    def load_params(
        self, path: str, node_or_namespace: str | Node = None
    ) -> dict[str, Any]:
        """Load parameters from a yaml file.

        If the config only contains a `ros__parameters` section the entire config is returned regardless of whether `node_or_namespace` was passed. Otherwise, if `node_or_namespace` is provided, the loaded config dict is searched for a matching section. If no matching section can be found a ValueError will be raised.

        The following wildcards are supported for parameter sections:
        * `/**`: matches any number of tokens, may be followed by additional tokens and a node name
        * `/*`: skips a single namespace token, or ignores the node's name if at the end

        Note that *better_launch* could not care less whether you put `ros__parameters` in your configs - if it is there it will be silently discarded.

        .. seealso::

            `ROS2 design doc on wildcards<https://github.com/ros2/design/blob/gh-pages/articles/160_ros_command_line_arguments.md#multiple-parameter-assignments>`_

        Parameters
        ----------
        path : str
            The path to the config. Will be resolved using :py:meth:`find`.
        node_or_namespace : str | Node, optional
            Used to specifiy which section of the config to return.

        Returns
        -------
        dict[str, Any]
            The key-value pairs from the config.

        Raises
        ------
        ValueError
            If the path cannot be resolved, if `node_or_namespace` is supplied and no matching section could be found, or if a substitution failed.
        IOError
            If the config file could not be read.
        """
        path = self.find(filename=path)

        with open(path) as f:
            params = yaml.safe_load(f)

        if "ros__parameters" in params:
            # Return the entire config if it doesn't contain sections for different nodes/namespaces
            return params["ros__parameters"]

        if node_or_namespace:
            ns = node_or_namespace
            if isinstance(ns, Node):
                ns = ns.fullname

            ns = ns.strip("/")

            # See https://github.com/ros2/design/blob/gh-pages/articles/160_ros_command_line_arguments.md
            def path_to_regex(path: str) -> str:
                parts = path.strip("/").split("/")
                regex_parts = []
                
                for part in parts:
                    if part == "**":
                        # Match any number of tokens
                        regex_parts.append(r".*")
                    elif part == "*":
                        # Match a single token
                        regex_parts.append(r"[^/]+")
                    else:
                        # Regular token
                        regex_parts.append(part)
                
                # Does NOT start with a slash as only the node name could be specified
                return "^" + "/".join(regex_parts) + "$"

            # According to the above design document, params files are not allowed to have nesting, 
            # (e.g. my_namespace/: other_namespace/node: ros__parameters), so no need to delve
            for key in params.keys():
                pattern = path_to_regex(key)
                if regex.match(pattern, ns) is not None:
                    params = params[key]
                    break
            else:
                # We didn't find any matching parameters, so this either doesn't contain a section 
                # for this node, or it is not a ros parameters file
                raise ValueError(f"No section matching full node name {ns}")
            
            if "ros__parameters" in params:
                params = params["ros__parameters"]

        return params

    def get_ros_message_type(self, message_string: str) -> type:
        """Loads a ROS2 message type from a string representation.

        Message representations must follow the pattern `<package>/<type>/<message>`, where
        * <package> is the ROS2 package that defines the message.
        * <type> is the type of message, typically one of `msg`, `srv` or `action`.
        * <message> is the name of the message itself with proper capitalization.

        Parameters
        ----------
        message_string : str
            A message representation of the form `<package>/<type>/<message>`.

        Returns
        -------
        type
            The message class.

        Raises
        ------
        ImportError
            If the message type could not be imported.
        """
        module_name, message_name = message_string.rsplit("/", maxsplit=1)
        module = importlib.import_module(module_name.replace("/", "."))
        return getattr(module, message_name)

    def subscriber(
        self,
        topic: str,
        message_type: type,
        callback: Callable[[Any], None],
        qos_profile: QoSProfile | int = 10,
    ) -> RosSubscriber:
        """Create a ROS2 subscriber to receive messages.

        Parameters
        ----------
        topic : str
            The topic to listen on for messages.
        message_type : str | type
            The type of the messages that will be received. Strings must follow the pattern `<package>/msg/<message>`.
        callback : Callable[[Any], Any]
            A function that will be called whenever a message is received.
        qos_profile : QoSProfile | int, optional
            A quality of service profile that changes how the publisher handles connections and retains data.

        Returns
        -------
        RosSubscriber
            The subscriber object. Although not required for Jazzy and below, it is recommended to keep a reference.
        """
        if isinstance(message_type, str):
            message_type = self.get_ros_message_type(message_type)

        return self.shared_node.create_subscriber(
            message_type,
            topic,
            callback,
            qos_profile=qos_profile,
        )

    def publisher(
        self, topic: str, message_type: str | type, qos_profile: QoSProfile | int = 10
    ) -> RosPublisher:
        """Create a ROS2 publisher using the :py:meth:`shared_node`.

        Parameters
        ----------
        topic : str
            The topic to publish messages on.
        message_type : str | type
            The message type that will be published. Strings must follow the pattern `<package>/msg/<message>`.
        qos_profile : QoSProfile | int, optional
            A quality of service profile that changes how the publisher handles connections and retains data.

        Returns
        -------
        RosPublisher
            The publisher object. Although not required for Jazzy and below, it is recommended to keep a reference.
        """
        if isinstance(message_type, str):
            message_type = self.get_ros_message_type(message_type)

        return self.shared_node.create_publisher(
            message_type,
            topic,
            qos_profile=qos_profile,
        )

    def publish_message(
        self,
        topic: str,
        message_type: str | type,
        message_args: dict[str, Any],
        qos_profile: QoSProfile | int = 10,
    ) -> None:
        """Convenience method to publish a single message. If you plan to publish additional messages, use :py:meth:`publisher` instead and use the instance.

        Parameters
        ----------
        topic : str
            The topic to publish messages on.
        message_type : str | type
            The message type that will be published. Strings must follow the pattern `<package>/msg/<message>`.
        message_args : dict[str, Any]
            The keyword arguments from which the message will be constructed.
        qos_profile : QoSProfile | int, optional
            A quality of service profile that changes how the publisher handles connections and retains data.
        """
        if isinstance(message_type, str):
            message_type: type = self.get_ros_message_type(message_type)

        pub = self.publisher(topic, message_type, qos_profile)
        msg = message_type(**message_args)
        pub.publish(msg)

    def service(
        self,
        topic: str,
        service_type: str | type,
        callback: Callable[[Any], Any],
        qos_profile: QoSProfile = None,
    ) -> RosServiceProvider:
        """Create a ROS2 service provider using the :py:meth:`shared_node`.

        Parameters
        ----------
        topic : str
            The topic the service will live on.
        service_type : str | type
            The service's message type. Strings must follow the pattern `<package>/srv/<message>`.
        callback : Callable[[Any], Any]
            The function that will handle any requests to the service. The type of the request will be of type `service_type.Request`.
        qos_profile : QoSProfile, optional
            A quality of service profile that changes how the service handles connections.

        Returns
        -------
        RosServiceProvider
            The service object. Although not required for Jazzy and below, it is recommended to keep a reference.
        """
        if isinstance(service_type, str):
            service_type = self.get_ros_message_type(service_type)

        if not qos_profile:
            qos_profile = qos_profile_services_default

        return self.shared_node.create_service(
            service_type,
            topic,
            callback,
            qos_profile=qos_profile,
        )

    def service_client(
        self,
        topic: str,
        service_type: str | type,
        timeout: float = 5.0,
        qos_profile: QoSProfile = None,
    ) -> RosServiceClient:
        """Create a ROS2 service client that can be used to call a service.

        Parameters
        ----------
        topic : str
            The service topic to post requests on.
        service_type : str | type
            The service's message type. Strings must follow the pattern `<package>/srv/<message>`.
        timeout : float, optional
            Time to wait for the service to become available. Ignored if <= 0.
        qos_profile : QoSProfile, optional
            A quality of service profile that changes how the service handles connections.

        Returns
        -------
        RosServiceClient
            The client object. Although not required for Jazzy and below, it is recommended to keep a reference.

        Raises
        ------
        TimeoutError
            If the service did not become available within the specified timeout.
        """
        if isinstance(service_type, str):
            service_type = self.get_ros_message_type(service_type)

        if not qos_profile:
            qos_profile = qos_profile_services_default

        client = self.shared_node.create_client(
            service_type, topic, qos_profile=qos_profile
        )
        if timeout > 0.0:
            if not client.wait_for_service(timeout):
                raise TimeoutError(
                    f"Service client timed out ({topic}, {service_type})"
                )
        return client

    def call_service(
        self,
        topic: str,
        service_type: str | type,
        request_args: dict[str, Any],
        timeout: float = 5.0,
        qos_profile: QoSProfile = None,
    ) -> Any:
        """Makes a single service request and returns the result. If you plan to make additional requests, use :py:meth:`service_client` instead.

        Parameters
        ----------
        topic : str
            The service topic to post requests on.
        service_type : str | type
            The service's message type. Strings must follow the pattern `<package>/srv/<message>`.
        request_args : dict[str, Any]
            The keyword arguments from which the request message will be constructed.
        timeout : float, optional
            Time to wait for the service to become available. Ignored if <= 0.
        qos_profile : QoSProfile, optional
            A quality of service profile that changes how the service handles connections.

        Returns
        -------
        Any
            The result of the service call of type `service_type.Request`.

        Raises
        ------
        TimeoutError
            If the service did not become available within the specified timeout.
        """
        if isinstance(service_type, str):
            service_type = self.get_ros_message_type(service_type)

        srv = self.service_client(topic, service_type, timeout, qos_profile)
        req = service_type.Request(**request_args)
        return srv.call(req)

    def action_server(
        self,
        topic: str,
        action_type: str | type,
        callback: Callable[[Any], Any],
        qos_profile: QoSProfile = None,
    ) -> "RosActionServer":
        """Create a ROS2 action server using the :py:meth:`shared_node`.

        Parameters
        ----------
        topic : str
            The topic namespace to provide the action interface on.
        action_type : str | type
            The type of the actions to be handled. Strings must follow the pattern `<package>/action/<message>`.
        callback : Callable[[Any], Any]
            A function that will handle incoming action requests. The type of the requests will be of type :py:func:`rclpy.action.server.ServerGoalHandle` and contain an `action_type.Goal`.
        qos_profile : QoSProfile, optional
            A quality of service profile that changes how the action server handles connections and retains data.

        Returns
        -------
        RosActionServer
            The action server object. Although not required for Jazzy and below, it is recommended to keep a reference.
        """
        from rclpy.action import ActionServer

        if isinstance(action_type, str):
            action_type = self.get_ros_message_type(action_type)

        if not qos_profile:
            qos_profile = qos_profile_services_default

        return ActionServer(
            self.shared_node,
            action_type,
            topic,
            callback,
            goal_service_qos_profile=qos_profile,
            result_service_qos_profile=qos_profile,
            cancel_service_qos_profile=qos_profile,
        )

    def action_client(
        self,
        topic: str,
        action_type: str | type,
        timeout: float = 5.0,
        qos_profile: QoSProfile = None,
    ) -> "RosActionClient":
        """Create a ROS2 action client to execute long-running actions.

        Parameters
        ----------
        topic : str
            The topic namespace on which the action interface is provided.
        action_type : str | type
            The type of actions the action server handles. Strings must follow the pattern `<package>/action/<message>`.
        timeout : float, optional
            Time to wait for the action server to become available. Ignored if <= 0.
        qos_profile : QoSProfile, optional
            A quality of service profile that changes how the action server handles connections and retains data.

        Returns
        -------
        RosActionClient
            The action client object. Although not required for Jazzy and below, it is recommended to keep a reference.

        Raises
        ------
        TimeoutError
            If the action server did not become available within the specified timeout.
        """
        # Lazy import, these add a lot of overhead
        from rclpy.action import ActionClient

        if isinstance(action_type, str):
            action_type = self.get_ros_message_type(action_type)

        if not qos_profile:
            qos_profile = qos_profile_services_default

        client = ActionClient(
            self.shared_node,
            action_type,
            topic,
            goal_service_qos_profile=qos_profile,
            result_service_qos_profile=qos_profile,
            cancel_service_qos_profile=qos_profile,
        )

        if timeout > 0.0:
            if not client.wait_for_server(timeout):
                raise TimeoutError(f"Action client timed out ({topic}, {action_type})")
        return client

    @contextmanager
    def group(
        self, namespace: str, remaps: dict[str, str] = None
    ) -> Generator[Group, None, None]:
        """Groups are used to bundle nodes into logical collections.

        As in ROS2, groups allow to collect nodes under a common namespace. Additionally, in *better_launch* groups can be used to define topic remaps for all nodes added to them.

        Groups are intended to be used as context objects and can be nested, e.g.

        .. code:: python

            bl = BetterLaunch()
            with bl.group("outer"):
                with bl.group("inner"):
                    # Node will live within "/outer/inner/"
                    bl.node("mypkg", "mynode.py", "mynode")


        Parameters
        ----------
        namespace : str
            The group's namespace.
        remaps : dict[str, str], optional
            Topic remaps that will apply to all nodes in this group or any descendant groups.

        Yields
        ------
        Generator[Group, None, None]
            Places the group on the group stack and yields it. Exiting the context will pop the group from the group stack.

        Raises
        ------
        RuntimeError
            If a group is created within a :py:meth:`compose` context.
        """
        if self._composition_node:
            raise RuntimeError("Cannot add groups inside a composition node")

        group = Group(self.group_tip, namespace, remaps=remaps)
        self.group_tip.add_group(group)
        self._group_stack.append(group)
        try:
            yield group
        finally:
            self._group_stack.pop()

    def node(
        self,
        package: str,
        executable: str,
        name: str,
        *,
        remaps: dict[str, str] = None,
        params: str | dict[str, Any] = None,
        cmd_args: list[str] = None,
        env: dict[str, str] = None,
        isolate_env: bool = False,
        log_level: int = logging.INFO,
        output_config: (
            Node.LogSink | dict[Node.LogSource, set[Node.LogSink]]
        ) = "screen",
        reparse_logs: bool = True,
        anonymous: bool = False,
        hidden: bool = False,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        autostart_process: bool = True,
        ros_waittime: float = 3.0,
        lifecycle_waittime: float = 0.01,
        lifecycle_target: LifecycleStage = LifecycleStage.ACTIVE,
    ) -> Node:
        """Create a new ROS2 node process. The bread and butter of every ROS setup!

        Note that this method also handles lifecycle nodes (they REALLY should have a common interface). Note that especially for lifecycle nodes you probably want `autostart_process == True`, otherwise there lifecycle management will not exist. With `autostart_process == True`, a lifecycle node will automatically advance to `lifecycle_target` once it is up. Otherwise you can also call :py:meth:`Node.start` later.

        The `ROS2 documentation <https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html>`_ can provide some additional information regarding `params`, `remaps`, and so on.

        Parameters
        ----------
        package : str
            The package providing the node.
        executable : str
            The executable that should be run.
        name : str
            The name you want the node to be known as.
        remaps : dict[str, str], optional
            Tells the node to replace any topics it wants to interact with according to the provided dict.
        params : str | dict[str, Any], optional
            Any ROS parameters you want to pass to the node. These are the args you would typically have to declare in your launch file. A string will be interpreted as a path to a yaml file which will be lazy loaded using :py:meth:`BetterLaunch.load_params`.
        cmd_args : list[str], optional
            Additional command line arguments to pass to the node.
        env : dict[str, str], optional
            Additional environment variables to set for the node's process. The node process will merge these with the environment variables of the better_launch host process unless :py:meth:`isolate_env` is True.
        isolate_env : bool, optional
            If True, the node process' env will not be inherited from the parent process and only those passed via `env` will be used. Be aware that this can result in many common things to not work anymore since e.g. keys like *PATH* will be missing.
        log_level : int, optional
            The minimum severity a logged message from this node must have in order to be published.
        output_config : Node.LogSink  |  dict[Node.LogSource, set[Node.LogSink]], optional
            How log output from the node should be handled. Sources are `stdout`, `stderr` and `both`. Sinks are `screen`, `log`, `both`, `own_log`, and `full`. See :py:class:`Node` for more details.
        reparse_logs : bool, optional
            If True, *better_launch* will capture the node's output and reformat it before publishing.
        anonymous : bool, optional
            If True, the node name will be appended with a unique suffix to avoid name conflicts.
        hidden : bool, optional
            If True, the node name will be prepended with a "_", hiding it from common listings.
        on_exit : Callable, optional
            A function to call when the node's process terminates (after any possible respawns).
        max_respawns : int, optional
            How often to restart the node process if it terminates.
        respawn_delay : float, optional
            How long to wait before restarting the node process after it terminates.
        use_shell : bool, optional
            If True, invoke the node executable via the system shell. While this gives access to the shell's builtins, this has the downside of running the node inside a "mystery program" which is platform and user dependent. Generally not advised.
        autostart_process : bool, optional
            If True, start the node process before returning from this function.
        ros_waittime : float, optional
            How long to wait for the node to register with ROS. This should cover the time between the process starting and the node initializing itself. Set negative to wait indefinitely. Will do nothing if `autostart_process` is False.
        lifecycle_waittime : float, optional
            How long to wait for the node's lifecycle management to come up. This should cover the time between the node initializing itself (see `ros_waittime`) and creating its additional topics and services. While neglible on modern computers, slower devices and embedded systems may experience a noticable delay here. Set negative to wait indefinitely. Will do nothing if `autostart_process` is False.
        lifecycle_target : LifecycleStage, optional
            The lifecycle stage to bring the node into after starting. Has no effect if `autostart_process` is False or if the node does not appear to be a lifecycle node after waiting `ros_waittime + lifecycle_waittime`.

        Returns
        -------
        Node
            The node object wrapping the node process.

        Raises
        ------
        RuntimeError
            If you try to add a node withing a :py:meth:`compose` context.
        """
        if self._composition_node:
            raise RuntimeError("Cannot add nodes inside a composition node")

        if anonymous:
            name = self.get_unique_name(name)

        if hidden and not name.startswith("_"):
            name = "_" + name

        # Assemble additional node remaps from our group branch
        g = self.group_tip
        remaps = g.assemble_remaps()
        if remaps:
            remaps.update(remaps)

        namespace = g.assemble_namespace()

        node = Node(
            package,
            executable,
            name,
            namespace,
            remaps=remaps,
            params=params,
            cmd_args=cmd_args,
            env=env,
            isolate_env=isolate_env,
            log_level=log_level,
            output_config=output_config,
            reparse_logs=reparse_logs,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
        )

        g.add_node(node)
        if autostart_process:
            node.start()

            if node.check_ros2_connected(ros_waittime):
                if node.check_lifecycle_node(lifecycle_waittime):
                    node.lifecycle.transition(lifecycle_target)

        return node

    @contextmanager
    def compose(
        self,
        name: str,
        language: str = "cpp",
        composer_mode: Composer.ComposerMode = "normal",
        *,
        component_remaps: dict[str, str] = None,
        composer_remaps: dict[str, str] = None,
        params: str | dict[str, Any] = None,
        cmd_args: list[str] = None,
        env: dict[str, str] = None,
        isolate_env: bool = False,
        log_level: int = logging.INFO,
        output_config: (
            Node.LogSink | dict[Node.LogSource, set[Node.LogSink]]
        ) = "screen",
        reparse_logs: bool = True,
        anonymous: bool = False,
        hidden: bool = False,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        autostart_process: bool = True,
        ros_waittime: float = 3.0,
    ) -> Generator[Composer, None, None]:
        """Creates a composer node which can be used to load composable components.

        This can be used as a context object, e.g.

        .. code:: python

            bl = BetterLaunch()
            with bl.compose("my-composer"):
                bl.component("my_package", "mystuff:TheComponentOfDreams", "normal-component")

        Parameters
        ----------
        name : str
            The name you want the composer to be known as.
        language : str, optional
            The programming language of the composer (and components) you want to use.
        composer_mode : Composer.ComposerMode, optional
            ROS2 provides special composers for components that need multithreading or should be isolated from the rest.
        component_remaps : dict[str, str], optional
            Any remaps you want to apply to all *components* loaded into this composer.
        composer_remaps : dict[str, str], optional
            Remaps you want to apply for the composer itself. Usually less useful (i.e. not at all).
        params : str | dict[str, Any], optional
            Any ROS parameters you want to pass to the composer itself. These are the args you would typically have to declare in your launch file. A string will be interpreted as a path to a yaml file which will be lazy loaded using :py:meth:`BetterLaunch.load_params`.
        cmd_args : list[str], optional
            Additional command line arguments to pass to the composer.
        env : dict[str, str], optional
            Additional environment variables to set for the composer's process. The composer process will merge these with the environment variables of the better_launch host process unless :py:meth:`isolate_env` is True.
        isolate_env : bool, optional
            If True, the composer process' env will not be inherited from the parent process and only those passed via `env` will be used. Be aware that this can result in many common things to not work anymore since e.g. keys like *PATH* will be missing.
        log_level : int, optional
            The minimum severity a logged message from this composer must have in order to be published.
        output_config : Node.LogSink  |  dict[Node.LogSource, set[Node.LogSink]], optional
            How log output from the composer should be handled. Sources are `stdout`, `stderr` and `both`. Sinks are `screen`, `log`, `both`, `own_log`, and `full`. See :py:class:`Node` for more details.
        reparse_logs : bool, optional
            If True, *better_launch* will capture the composer's output and reformat it before publishing.
        anonymous : bool, optional
            If True, the composer name will be appended with a unique suffix to avoid name conflicts.
        hidden : bool, optional
            If True, the composer name will be prepended with a "_", hiding it from common listings.
        on_exit : Callable, optional
            A function to call when the composer's process terminates (after any possible respawns).
        max_respawns : int, optional
            How often to restart the composer process if it terminates.
        respawn_delay : float, optional
            How long to wait before restarting the composer process after it terminates.
        use_shell : bool, optional
            If True, invoke the composer executable via the system shell. While this gives access to the shell's builtins, this has the downside of running the composer inside a "mystery program" which is platform and user dependent. Generally not advised.
        autostart_process : bool, optional
            If True, start the composer process before returning from this function. Note that setting this to False for a composer will make it unusable as a context object, since you won't be able to load any components.
        ros_waittime : float, optional
            How long to wait for the composer to register with ROS. This should cover the time between the process starting and the composer initializing itself. Set negative to wait indefinitely. Will do nothing if `autostart_process` is False.

        Yields
        ------
        Generator[Composer, None, None]
            Sets the composition flag and yields the composer.

        Raises
        ------
        RuntimeError
            If you try to create a composer within a :py:func:`compose` context.
        """
        if self._composition_node is not None:
            raise RuntimeError("Cannot nest composition nodes")

        if anonymous:
            name = self.get_unique_name(name)

        if hidden and not name.startswith("_"):
            name = "_" + name

        # NOTE: Remaps apply to the components, not the composer
        g = self.group_tip
        remaps = g.assemble_remaps()
        if component_remaps:
            remaps.update(component_remaps)

        namespace = g.assemble_namespace()

        comp = Composer(
            name,
            namespace,
            language,
            composer_mode,
            component_remaps=remaps,
            composer_remaps=composer_remaps,
            params=params,
            cmd_args=cmd_args,
            env=env,
            isolate_env=isolate_env,
            log_level=log_level,
            output_config=output_config,
            reparse_logs=reparse_logs,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
        )

        try:
            g.add_node(comp)

            if autostart_process:
                comp.start()
                comp.check_ros2_connected(ros_waittime)

            self._composition_node = comp
            yield comp
        finally:
            self._composition_node = None

    def component(
        self,
        package: str,
        plugin: str,
        name: str,
        *,
        remaps: dict[str, str] = None,
        params: str | dict[str, Any] = None,
        use_intra_process_comms: bool = True,
        ros_waittime: float = 3.0,
        lifecycle_waittime: float = 0.01,
        lifecycle_target: LifecycleStage = LifecycleStage.ACTIVE,
        **extra_composer_args: dict[str, Any],
    ) -> Component:
        """Create a component and load it into an existing :py:meth:`compose` context.

        If you instead want to load components without a `compose` context, you should instantiate :py:class:`Component` objects directly, then load them via :py:meth:`Component.start` or :py:meth:`Composer.load_component`.

        Parameters
        ----------
        package : str
            The package providing the component implementation.
        plugin : str
            The name the component is registered as, typically of the form `<package>::<Name>`.
        name : str
            The name the instantiated component should be known as.
        remaps : dict[str, str], optional
            Tells the node to replace any topics it wants to interact with according to the provided dict.
        params : str | dict[str, Any], optional
            Any ROS parameters you want to pass to the component. These are the args you would typically have to declare in your launch file. A string will be interpreted as a path to a yaml file which will be lazy loaded using :py:meth:`BetterLaunch.load_params`.
        use_intra_process_comms : bool, optional
            If True, ask the composer node to enable intra-process communication, i.e. share memory between components when passing messages instead of serializing and deserializing.
        ros_waittime : float, optional
            How long to wait for the component to register with ROS. This should cover the time between the process starting and the component initializing itself. Set negative to wait indefinitely. Will do nothing if `autostart_process` is False.
        lifecycle_waittime : float, optional
            How long to wait for the component's lifecycle management to come up. This should cover the time between the component initializing itself (see `ros_waittime`) and creating its additional topics and services. While neglible on modern computers, slower devices and embedded systems may experience a noticable delay here. Set negative to wait indefinitely. Will do nothing if `autostart_process` is False.
        lifecycle_target : LifecycleStage, optional
            The lifecycle stage to bring the component into after starting. Has no effect if `autostart_process` is False or if the component does not appear to be a lifecycle component after waiting `ros_waittime + lifecycle_waittime`.

        Returns
        -------
        Component
            The component that has been loaded into the current :py:meth:`compose` context.

        Raises
        ------
        RuntimeError
            If this is called outside a :py:meth:`compose` context.
        """
        if self._composition_node is None:
            raise RuntimeError("Cannot add component outside a compose() node")

        comp = Component(
            self._composition_node,
            package,
            plugin,
            name,
            remaps=remaps,
            params=params,
        )

        # Equivalent to self._composition_node.load_component(comp)
        comp.start(
            use_intra_process_comms=use_intra_process_comms,
            **extra_composer_args,
        )

        if comp.check_ros2_connected(ros_waittime):
            if comp.check_lifecycle_node(lifecycle_waittime):
                comp.lifecycle.transition(lifecycle_target)

        return comp

    @overload
    def include(
        self, launchfile: str, *, pass_launch_func_args: bool = True, **kwargs
    ) -> None: ...

    @overload
    def include(
        self,
        package: str,
        launchfile: str,
        *,
        pass_launch_func_args: bool = True,
        **kwargs,
    ) -> None: ...

    def include(
        self,
        *search_args: str,
        pass_launch_func_args: bool = True,
        **kwargs,
    ) -> None:
        """Include another launch file, resolving its path using :py:meth:`find`.

        The file is first read into memory and checked. If it seems to be a *better_launch* launch file, it is executed immediately (using :py:func:`exec`). The BetterLaunch instance and global context will be shared. Any arguments to :py:meth:`launch_this` in the included launch file will be ignored.

        If the file does not appear to be a *better_launch* launch file, it is assumed to be a regular ROS2 launch file. In this case a :py:class:`launch.actions.IncludeLaunchDescription` instance is created and passed to :py:meth:`ros2_actions`.

        Parameters
        ----------
        package : str
            The package containing the specified launch file.
        launchfile : str
            The name of a launch file to execute.
        pass_launch_func_args : bool, optional
            If True, all :py:meth:`launch_args` will be passed to the included launch file. Additional launch arguments can also be provided via the `kwargs`.

        Raises
        ------
        ValueError
            If the passed in `search_args` cannot be handled.
        """
        if len(search_args) == 1:
            package = None
            launchfile = search_args[0]
        elif len(search_args) == 2:
            package, launchfile = search_args
        else:
            raise ValueError(f"Incorrect number of arguments: {search_args}")

        # Pass additional arguments, e.g. launch args
        include_args = {}
        if pass_launch_func_args:
            include_args.update(self.launch_args)
        include_args.update(**kwargs)

        file_path = self.find(filename=launchfile, package=package)

        if find_launchthis_function(file_path):
            try:
                source = open(file_path).read()
                code = compile(source, launchfile, "exec")

                # Make sure the included launch file reuses our BetterLaunch instance
                global_args = dict(globals())
                global_args[_bl_singleton_instance] = self
                global_args[_bl_include_args] = include_args

                # Since we're running an entire module locals won't have any effect
                exec(code, global_args)
            except Exception as e:
                self.logger.error(
                    f"Launch include '{package}/{launchfile}' failed: {e}"
                )
                raise
        else:
            # TODO could be stricter about verification here
            # Was not a better_launch launch file, assume it's a ROS2 launch file (py, xml, yaml)
            self._include_ros2_launchfile(file_path, **include_args)

    def _include_ros2_launchfile(self, file_path, **kwargs) -> None:
        # Delegate to ros2 launch service
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import (
            AnyLaunchDescriptionSource,
        )

        # See https://github.com/ros2/launch_ros/blob/rolling/ros2launch/ros2launch/api/api.py#L175
        ros2_include = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(file_path),
            launch_arguments=[
                # ROS2 can handle only tuples of strings and strings/substitutions here...
                (key, str(val) if val is not None else "")
                for key, val in kwargs.items()
            ],
        )
        self.ros2_actions(ros2_include)

    def ros2_launch_service(
        self,
        name: str = "LaunchService",
        launchservice_args: list[str] = None,
        start_immediately: bool = True,
    ) -> Ros2LaunchWrapper:
        """Create or retrieve a manager object that can be used for queueing ROS2 launch actions.

        Usually, calling :py:meth:`ros2_actions` is more convenient for queueing actions. However, calling this *first* allows to prevent starting the underlying :py:class:`launch.LaunchService` immediately, giving more control over when the actions are executed.

        Since the `LaunchService` insists on running on the main thread it will be started as a sub process.

        Note that only one instance of the ROS2 wrapper should ever exist. Calling this method after it has been created will return the already existing instance instead. Any passed arguments will be silently discarded.

        Parameters
        ----------
        name : str, optional
            The name used to identify the process and its logger.
        launchservice_args : list[str], optional
            Additional launch arguments to pass to the ROS2 launch service. These will end up in :py:meth:`launch.LaunchContext.argv`.
        start_immediately : bool, optional
            If True, the ROS2 launch service process is started immediately.

        Returns
        -------
        Ros2LaunchWrapper
            The wrapper hosting the ROS2 launch service process.
        """
        if not self._ros2_launcher:
            self._ros2_launcher = Ros2LaunchWrapper(
                name=name, launchservice_args=launchservice_args
            )

        if start_immediately:
            self._ros2_launcher.start()

        return self._ros2_launcher

    def ros2_actions(self, *ros2_actions) -> Ros2LaunchWrapper:
        """Submit additional ROS2 launch actions for execution.

        If no :py:class:`launch.LaunchService` exists yet it will be created and started immediately.
        """
        self.ros2_launch_service().queue_ros2_actions(*ros2_actions)
        return self._ros2_launcher

    def run_later(self, delay: float, callback: Callable, *args, **kwargs) -> Future:
        """Convenience method for running a callback with a delay. The callback will be called on a separte thread.

        This mainly exists to cover the use case where you want to interact with ROS from an `rclpy.Timer`. A synchronous call from within a timer (e.g. a service call like :py:meth:`Node.set_live_params`) will block ROS' background event loop, preventing publishers, subscribers, services, etc. from doing their work. It will also prevent a clean shutdown as ROS usually waits for the event queue to become empty.

        When executing a long running task this way it is a good idea to check :py:meth:`is_shutdown` in between iterations.

        Parameters
        ----------
        delay : float
            How long to wait in seconds before calling the callback.
        callback : Callable
            The function to call after the timeout. Will not be called if :py:meth:`shutdown` is called beforehand. 
        *args : Any, optional
            Positional arguments to the callback.
        **kwargs : Any, optional
            Keyword arguments to the callback.

        Returns
        -------
        Future
            A future which will be cancelled on `shutdown`, otherwise it will hold the callback's result or an exception the callback raised.
        """
        future = Future()

        def run():
            time.sleep(delay)

            if future.cancelled():
                return

            if self.is_shutdown:
                future.cancel()
                return

            try:
                ret = callback(*args, **kwargs)
                future.set_result(ret)
            except Exception as e:
                future.set_exception(e)

        # TODO this is a good candidate for running on an asyncio loop
        threading.Thread(target=run, daemon=True).start()
        return future
