import inspect
from contextlib import contextmanager
from collections import deque

from elements import Group, Composer, Node


def launch_this(launch_func):
    # TODO should make the function compatible with the ros2 launch system, e.g.
    # declare arguments, create a stub launch description, etc. Also should expose 
    # some kind of interface so that it can be included by other better_launch 
    # scripts.
    def generate_launch_description(_better_launch_instance = None):
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

    return generate_launch_description


class _BetterLaunchMeta(type):
    # Allows reusing an already existing BetterLaunch instance. 
    # Important for launch file includes.
    def __call__(cls, *args, **kwargs):
        existing_instance = kwargs.pop("_better_launch_instance", None)
        if existing_instance:
            return existing_instance
        
        obj = cls.__new__(cls, *args, **kwargs)
        obj.__init__(*args, **kwargs)
        return obj


class BetterLaunch(metaclass=_BetterLaunchMeta):
    def __init__(self, ns: str = "/", launch_args: dict = None):
        if launch_args is None:
            frame = inspect.currentframe()
            try:
                # Get the caller's locals
                launch_args = {k:v for k,v in frame.f_back.f_locals if not k.startswith("_")}
            finally:
                del frame

        self.all_args = launch_args

        if ns is None:
            ns = "/"
        elif not ns.startswith("/"):
            ns = "/" + ns

        self._container_stack = deque()
        self._container_stack.append(Group(ns))

    def find(self, pkg: str, filename: str = None, dir: str = None):
        # TODO
        pass

    def node(self, pkg: str, exec: str, name: str = None, *, anonymous: bool = False, **kwargs):
        # TODO establish namespace and remaps for nested groups
        for container in reversed(self._container_stack):
            if isinstance(container, Group):
                return container.add_node(pkg, exec, name, anonymous = anonymous, **kwargs)
        else:
            raise RuntimeException("Launcher did not contain a group, this should never happen!")

    def component(self, pkg: str, plugin: str, **kwargs):
        for container in reversed(self._container_stack):
            # Run inside the most recent composable container, if available
            if isinstance(container, Composer):
                return container.add_component(pkg, plugin, name=name, **kwargs)
        else:
            raise RuntimeException("Tried to run a composable without a composer")

    def include(self, pkg: str, launch_file: str, pass_all_args: bool = True, **kwargs):
        global_args = dict(globals())
        if pass_all_args:
            local_args = self.all_args
        else:
            local_args = kwargs

        file_path = ...
        if launch_file.endswith(".py"):
            # Check if the launch file uses better_launch
            with open(file_path) as f:
                content = f.read()
                if "better_launch" in content:
                    try:
                        code = compile(content, launch_file, "exec")
                        # TODO mechanism for passing instance not fully defined yet
                        global_args["_better_launcher_instance"] = self
                        exec(code, global_args, local_args)
                        return
                    except Exception as e:
                        # TODO log
                        raise e
        
        # Assume a normal ros2 launch file
        # TODO

    @contextmanager
    def group(self, ns: str = None, remap: dict[str, str] = None):
        group = Group(self, ns, remap)
        self._container_stack.append(group)
        try:
            yield group
        finally:
            self._container_stack.pop()

    @contextmanager
    def compose(self, name: str, language: str = "cpp"):
        composer = Composer(self, name, language)
        self._container_stack.append(composer)
        try:
            yield composer
        finally:
            self._container_stack.pop()

    # TODO Common stuff like joint_state_publisher, robot_state_publisher, moveit, rviz, gazebo
