from better_launch import BetterLaunch, launch_this


@launch_this
def launch(
    main_node_name: str,
    samples: int = 100,
    freq: float = 20,
    start_all: bool = True,
    *,
    **kwargs
):
    # Also offer a "BetterLaunchDescription" class that generates the launch description instead of 
    # starting everything immediately
    bl = BetterLaunch()

    with bl.group(ns="test"):
        bl.node("mypkg", "test.py", main_node_name)

        if start_all:
            bl.include("otherpkg", "otherpkg.launch.py", **bl.all_args)

        with bl.compose():
            # Automatically detects
            bl.component("mypkg", "mypkg::Composable")
            bl.component("myotherpkg", "mypotherkg::Composable")

        bl.lifecycle_node("my_lifecycle_pkg", "lifecycle_node.py", target_state=bl.LIFECYCLE_STARTED)
