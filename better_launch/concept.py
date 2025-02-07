from better_launch import BetterLaunch, launch_this


# NOTE arguments default to NONE if not specified
@launch_this(ui=True)
def test(enable_x: bool):
    """
    This is how nice your launch files could be!
    """
    bl = BetterLaunch()

    with bl.group("test"):
        bl.node(
            "examples_rclpy_minimal_publisher",
            "publisher_local_function",
            "test_node",
        )

    with bl.compose("composed"):
        # TODO should implement a Component class so it can be used to manipulate components
        bl.component("composition", "composition::Talker", "comp_talker")
        bl.component("composition", "composition::Listener", "comp_listener")
