from better_launch import BetterLaunch, launch_this


@launch_this
def test():
    """
    This is how nice your launch files could be!
    """
    bl = BetterLaunch()

    with bl.group("test"):
        bl.node(
            "examples_rclpy_minimal_publisher",
            "publisher_local_function",
            "my_publisher",
        )
        bl.node(
            "examples_rclpy_minimal_subscriber",
            "subscriber_member_function",
            "my_listener",
        )
