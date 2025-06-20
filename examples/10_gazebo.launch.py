#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this, gazebo


@launch_this
def over_stimulation():
    """
    An example for starting a very simple gazebo simulation, consisting of an empty world into 
    which we spawn a cube model.
    """
    # Still need to instantiate BetterLaunch first
    bl = BetterLaunch()
    bl.logger.info("Gazebo version: %s", gazebo.get_gazebo_version())

    # TODO test topic bridges
    gazebo.gazebo_launch("better_launch", "test.world")
    gazebo.spawn_model("cube", bl.find("better_launch", "cube.sdf"))
    gazebo.spawn_world_transform()

    bl.logger.info("The loaded gazebo world is %s", gazebo.GazeboBridge.world())
