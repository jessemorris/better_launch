from better_launch import BetterLaunch, launch_this


# UI will be ignored on included files
@launch_this(ui=True)
# enable_x will be filled from the original launch file if include() was called with 
# pass_launch_func_args == True
def test(enable_x: bool):
    # This will reuse the original instance
    bl = BetterLaunch()

    # This group will be embedded according to how the include was placed
    with bl.group("lifecycle_B"):
        bl.node(
            "lifecycle",
            "lifecycle_listener",
            "lifecycle_listener",
        )

        # Include a ROS2 launch file. These nodes will not be managed by BetterLaunch (for now)
        bl.include("composition_demo.launch.py", "composition")
