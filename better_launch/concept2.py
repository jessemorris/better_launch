from better_launch import BetterLaunch, launch_this, LifecycleStage


# NOTE arguments default to NONE if not specified
@launch_this(ui=True)
def test(enable_x: bool):
    """
    This is how nice your launch files could be!
    """
    bl = BetterLaunch()

    #with bl.group("test"):
    #    bl.node(
    #        "examples_rclpy_minimal_publisher",
    #        "publisher_local_function",
    #        "test_node",
    #    )

    #with bl.group("composition"):
    #    with bl.compose("composed"):
    #        bl.component("composition", "composition::Talker", "comp_talker")
    #        bl.component("composition", "composition::Listener", "comp_listener")

    with bl.group("lifecycle_B"):
        bl.node(
            "lifecycle",
            "lifecycle_listener",
            "lifecycle_listener",
        )
