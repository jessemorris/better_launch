from better_launch import BetterLaunch, launch_this
from better_launch.elements import Component


@launch_this(ui=True)
def not_a_song():
    """
    This example will create a Composer node and load two example plugins into it from the composition package:
    * composition/composition::Talker
    * composition/composition::Listener

    Using the terminal UI (the TUI), you will see how they appear as a single node. Opening the node dialog from the sidebar will also allow you to unload individual components or kill the entire composer.
    """
    bl = BetterLaunch()

    with bl.compose("my_composer") as composer:
        # We load the first component immediately
        bl.component("composition", "composition::Talker", "comp_talker")

    # Since we exited the composer context, we can't use bl.component anymore,
    # but we can still use the composer later
    comp2 = Component(
        composer, 
        "composition", 
        "composition::Listener", 
        "comp_listener"
    )
    comp2.start()
    # composer.load_component(comp2) would also work
