from better_launch import BetterLaunch, launch_this, LifecycleStage


@launch_this(ui=True)
def test():
    """In this example, two separate nodes that support lifecycle management will be started:
    * lifecycle/lifecycle_talker
    * lifecycle/lifecycle_listener

    The talker will be started in the PRISTINE stage, which means that its process will be started, but the node itself will idle in its UNCONFIGURED state. Using the TUI's node menu from the sidebar, you can then ask the talker node to transition into its ACTIVE stage.
    """
    bl = BetterLaunch()

    with bl.group("test"):
        bl.node(
            "lifecycle",
            "lifecycle_talker",
            "lc_talker",
            lifecycle_target=LifecycleStage.PRISTINE
        )
        bl.node(
            "lifeycle",
            "lifecycle_listener",
            "lc_listener",
        )
