from better_launch import BetterLaunch, launch_this


@launch_this
def test(enable_listener: bool = False):
    """
    When writing a better_launch launch file, every argument of your launch function will be exposed on the command line. When running the launch file either directly or via `bl` you can pass this argument as follows:

    .. code:: bash

        bl better_launch 04_launch_arguments.py --enable_listener True

    These arguments can be used directly in your launch code without the need for conditions and substitutions. And in case you want to know what your launch file can actually do, you can always pass `--help` to it - try it out!

    Parameters
    ----------
    enable_listener : bool, optional
        Whether to start the listener node.
    """
    bl = BetterLaunch()

    with bl.group("test"):
        bl.node(
            "examples_rclpy_minimal_publisher",
            "publisher_local_function",
            "my_publisher",
        )

        # Yay, no more clunky condition substitutions!
        if enable_listener:
            bl.node(
                "examples_rclpy_minimal_subscriber",
                "subscriber_member_function",
                "my_listener",
            )
