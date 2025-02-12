import os
from better_launch import BetterLaunch, launch_this, LifecycleStage


@launch_this(ui=True)
# All arguments will be exposed as command line args (e.g. --enable_x)
# NOTE arguments default to NONE if not specified
def test(enable_x: bool):
    # The function docstring will be used to create the CLI help message
    """
    This is how nice your launch files could be!
    """
    bl = BetterLaunch()

    # Regular node
    #with bl.group("test"):
    #    bl.node(
    #        "examples_rclpy_minimal_publisher",
    #        "publisher_local_function",
    #        "test_node",
    #    )

    # Composition
    #with bl.group("composition"):
    #    with bl.compose("composed"):
    #        bl.component("composition", "composition::Talker", "comp_talker")
    #        bl.component("composition", "composition::Listener", "comp_listener")

    # Lifecycle nodes
    with bl.group("lifecycle_A"):
        bl.node(
            "lifecycle",
            "lifecycle_talker",
            "lifecycle_talker",
            lifecycle_target=LifecycleStage.PRISTINE,
        )

        # Include another BetterLaunch file within this group
        bl.include(os.path.dirname(__file__) + "/example2.py")
