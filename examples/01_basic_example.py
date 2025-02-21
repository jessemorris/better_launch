from better_launch import BetterLaunch, launch_this


@launch_this
def first_steps():
    """
    This is how nice your launch files could be!

    You can run this launch file either directly or with the included `bl` script, which should be on your PATH once you have built better_launch and sourced your workspace:

    .. code:: bash

        bl better_launch 01_basic_example.py
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
