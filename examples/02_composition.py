from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def test():
    """
    This example will create a Composer node and load two example plugins into it from the composition package:
    * composition/composition::Talker
    * composition/composition::Listener
    
    Using the terminal UI (the TUI), you will see how they appear as a single node. Opening the node dialog from the sidebar will allow you to unload individual components or kill the entire composer.
    """
    bl = BetterLaunch()

    with bl.group("composition"):
        with bl.compose("my_composer"):
            bl.component("composition", "composition::Talker", "comp_talker")
            bl.component("composition", "composition::Listener", "comp_listener")
