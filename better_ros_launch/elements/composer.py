from .node import execute_node


# TODO should composer inherit from group?
class Composer:
    def __init__(self, launcher, name: str, language: str):
        # NOTE: does not support referencing an already existing instance. If you want to reuse
        # the container, just keep a reference to it.
        self.launcher = launcher
        self.name = name
        self.language = language

    def start(self):
        package = f"rcl{self.language}_components"
        executable = "component_container"
        execute_node(self.launcher, package, executable, name=self.name)

    def add_component(self, package: str, plugin: str, **kwargs):
        # TODO call the load_node service
        # TODO handle group namespaces, remaps, etc.
        # See https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/load_composable_nodes.py
        pass
