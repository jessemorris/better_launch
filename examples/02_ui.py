from better_launch import BetterLaunch, launch_this


# NOTE This is the new part!
@launch_this(ui=True)
def a_nice_ui():
    """
    This example starts the same nodes as the previous one, but uses a terminal UI for display and management. Use the keyboard or mouse to interact with it and see what you can do! 
    """
    bl = BetterLaunch()

    with bl.group("basic"):
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
