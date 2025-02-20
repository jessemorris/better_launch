from better_launch import BetterLaunch, launch_this


@launch_this(ui=False)
def test(x: int = 2, y: int = 2, theta: float = 0.2):
    """
    better_launch can include both its own launch files and ROS2 launch files (.py, .yaml, .xml). When including ROS2 launch actions, better_launch creates a child process which will run the ROS2 launch service instance which will handle the passed actions asynchronously in the background.

    Unfortunately, due to their ludicrous complexity it is not feasible to digest and analyze what these actions are doing, especially when better_launch was written as an alternative. For this reason, anything started via the ROS2 launch system by better_launch will appear under a single 'LaunchService' node in the TUI.
    """
    bl = BetterLaunch()

    # If no package is given the current launch file's package is used!
    # TODO something in the logging fails
    bl.include("ros2_turtlesim.launch.py", turtlesim_ns="my_turtlesim")

    # Just for the sake of this tutorial, we could also import it as usual and get proper type hints
    Spawn = bl.get_ros_message_type("turtlesim/srv/Spawn")

    # TODO not working yet
    # Since better_launch executes actions immediately, you can also interact with e.g. nodes immediately!
    # client = bl.service_client(
    #     "/my_turtlesim/spawn",
    #     Spawn,
    #     timeout=5.0
    # )

    # req = Spawn.Request(x=x, y=y, theta=theta)
    # client.call(req)
