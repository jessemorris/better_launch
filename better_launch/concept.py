from better_launch import BetterLaunch, launch_this


#@launch_this
def concept(
    main_node_name: str,
    samples: int = 100,
    freq: float = 20,
    start_all: bool = True,
):
    # Also offer a "BetterLaunchDescription" class that generates the launch description instead of 
    # starting everything immediately
    bl = BetterLaunch()

    with bl.group(ns="test"):
        bl.node("mypkg", "test.py", main_node_name)

        if start_all:
            bl.include("otherpkg", "otherpkg.launch.py", **bl._launch_args)

        with bl.compose():
            # Automatically detects
            bl.component("mypkg", "mypkg::Composable")
            bl.component("myotherpkg", "myotherpkg::Composable")

        bl.lifecycle_node("my_lifecycle_pkg", "lifecycle_node.py", target_state=bl.LIFECYCLE_STARTED)


# NOTE arguments default to NONE if not specified
# TODO do required arguments make sense for launch files?
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
