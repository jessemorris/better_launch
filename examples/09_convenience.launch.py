#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this, convenience


@launch_this
def so_comfortable(robot_description: str = None, urdf: str = None):
    """
    The convenience module provides shortcuts for a couple of tasks that are common enough to be nice to have, yet too specific to include in the main API.
    """
    # Still need to instantiate BetterLaunch first
    bl = BetterLaunch()
    
    if robot_description and urdf:
        convenience.robot_state_publisher(robot_description, urdf)

    convenience.rviz()
