from better_launch import BetterLaunch, launch_this

@launch_this
def great_atuin(
    turtlesim_ns: str = "turtlesim1", 
    use_provided_red: bool = True, 
    new_background_r: int = 200,
):
    """This launch file implements the same functionality as the previous turtlesim example, but using better_launch functionality instead of ROS2.

    Parameters
    ----------
    turtlesim_ns : str, optional
        Namespace of the turtlesim node.
    use_provided_red : bool, optional
        Allow changing the background color.
    new_background_r : int, optional
        Must be 200 to do anything.
    """
    bl = BetterLaunch()

    with bl.group(turtlesim_ns):
        turtle_node = bl.node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim",
            # Pass parameters directly 
            params={"background_r": 120}
        )

        bl.call_service(
            topic=f"/{turtlesim_ns}/spawn",
            service_type="turtlesim/srv/Spawn",
            request_args={"x": 2.0, "y": 2.0, "theta": 0.2},
        )

        if use_provided_red:
            def update_background():
                bl.logger.info("Updating background_r")
                turtle_node.set_live_params({"background_r": new_background_r})
                timer.cancel()

            timer = bl.shared_node.create_timer(2.0, update_background)
