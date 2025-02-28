# TODO Create convenience module for common stuff like joint_state_publisher, robot_state_publisher, moveit, rviz, gazebo (see simple_launch)
import os
import threading
import time
from better_launch import BetterLaunch
import subprocess
from .utils.gazebo import GazeboBridge, get_gazebo_prefix, gazebo_launch_setup
from tempfile import NamedTemporaryFile
from better_launch.ros import logging as roslog


def joint_state_publisher(use_gui=True, **params) -> None:
    """
    Adds a joint_state_publisher or joint_state_publisher_gui with passed arguments as parameters.

    Parameters
    ----------
    use_gui : bool, optional
        Whether to use the GUI version of the joint_state_publisher (default is True).
    **params : dict
        Additional parameters to pass to the node (e.g., name, remaps, params, etc.).

    Returns
    -------
    None
        This function does not return anything.
    """
    launcher = BetterLaunch.instance()
    
    if use_gui:
        launcher.node(
            package="joint_state_publisher_gui", 
            executable="joint_state_publisher_gui", 
            **params
        )
    else:
        launcher.node(
            package="joint_state_publisher", 
            executable="joint_state_publisher", 
            **params
        )



def rviz(config_file=None, warnings=False):
    """
    Runs RViz with the given config file and optional warning level suppression.
    If a config file is specified, it attempts to resolve the path using launcher.find and uses the resolved path.
    """
    launcher = BetterLaunch.instance()
    args = []
    if config_file:
        config_file = launcher.find(config_file)
        args += ["-d", config_file]
    if not warnings:
        args.extend(["--ros-args", "--log-level", "FATAL"])
    launcher.node(package="rviz2", executable="rviz2", cmd_args=args)


def moveit(config_pkg: str, config_file: str):
    """
    Launches a MoveIt configuration specified by package and launch file path
    """
    launcher = BetterLaunch.instance()
    moveit_launch_file_path = launcher.find(config_pkg, f"{config_file}.launch.py")
    launcher.include(config_pkg, moveit_launch_file_path)


def robot_description(
    package=None,
    description_file=None,
    description_dir=None,
    xacro_args=None,
):
    """
    Returns the robot description after potential xacro parse if the file ends with xacro or xacro args are defined.
    """
    launcher = BetterLaunch.instance()
    name = os.path.basename(BetterLaunch._launchfile)
    logger = roslog.get_logger(name)
    description_file = launcher.find(package, description_file, description_dir)
    if description_file.endswith("urdf") and xacro_args is None:
        with open(description_file) as f:
            urdf_xml = f.read()
        return urdf_xml
    cmd = ["xacro", description_file] + (
        xacro_args if isinstance(xacro_args, list) else [xacro_args]
    )
    try:
        with subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        ) as proc:
            stdout, stderr = proc.communicate()
            if proc.returncode == 0:
                return stdout
            else:
                logger.warning(f"Error processing xacro: {stderr}")
                return None
    except Exception as e:
        logger.warning(f"Failed to execute xacro command: {e}")
        return None


def robot_state_publisher(
    package=None,
    description_file=None,
    description_dir=None,
    xacro_args=None,
    **node_args,
):
    """
    Add a robot state publisher node to the launch tree using the given description (URDF/Xacro) file.
    Incorporates robot_description into node_args directly for simplified node launching.

    Args:
    - package: Name of the package containing the description file; if None, an absolute path is assumed.
    - description_file: Name of the URDF/Xacro file.
    - description_dir: Directory containing the file; if None, the location is derived.
    - xacro_args: Arguments passed to the Xacro processor.
    - node_args: Additional node arguments such as remappings or parameters.
    """
    launcher = BetterLaunch.instance()
    name = os.path.basename(BetterLaunch._launchfile)
    logger = roslog.get_logger(name)

    try:
        urdf_xml = robot_description(
            launcher, package, description_file, description_dir, xacro_args
        )
        node_args["robot_description"] = urdf_xml
        launcher.node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            params=node_args,
        )
    except FileNotFoundError as e:
        logger.warning(f"Error loading robot description: {e}")
    except Exception as e:
        logger.warning(f"Error launching robot state publisher node: {e}")


def declare_gazebo_axes(axes={}):
    """
    Declares classical Gazebo axes as launch arguments.
    If axes is void then declares all 6 axes with default value 0.
    Otherwise declares the given axes with the given defaults.
    """
    gz_axes = ("x", "y", "z", "yaw", "pitch", "roll")
    if axes:
        filtered_axes = [axis for axis in gz_axes if axis in axes]
        declared_axes = {axis: axes[axis] for axis in filtered_axes}
    else:
        declared_axes = {axis: 0.0 for axis in gz_axes}

    return declared_axes


def gazebo_axes_args(declared_axes=None):
    """
    Constructs a list of command line arguments for a Gazebo simulation based on declared axes.
    If declared_axes is None, the axes are declared with default values.

    Returns:
    - list: A list of arguments suitable for a command-line interface.
    """
    if declared_axes is None:
        declared_axes = declare_gazebo_axes()
    args = []
    for axis, value in declared_axes.items():
        args.append(f"--{axis}")
        args.append(str(value))

    return args


def create_gazebo_bridge(*bridges: GazeboBridge, name="gz_bridge"):
    """
    Create a ros_gz_bridge::parameter_bridge with the passed GazeboBridge instances.
    """
    launcher = BetterLaunch.instance()
    if len(bridges) == 0:
        return

    ros_gz = "ros_" + get_gazebo_prefix()

    
    image_bridges = [bridge for bridge in bridges if bridge.is_image]

    std_config = sum([bridge.yaml() for bridge in bridges if not bridge.is_image], [])

    if std_config:
        dst = NamedTemporaryFile().name
        with open(dst, "w") as file:
            file.write(std_config)

        launcher.load_params(dst)
        launcher.node(
            package=f"{ros_gz}_bridge",
            executable="parameter_bridge",
            name=name,
            params={"parameters": {"config_file": dst}},
        )

    if image_bridges:
        remappings = []
        for bridge in image_bridges:
            for ext in ("", "/compressed", "/compressedDepth", "/theora"):
                remapped_gz_topic = f"{bridge.gz_topic}{ext}"
                remapped_ros_topic = f"{bridge.ros_topic}{ext}"
                remappings.append((remapped_gz_topic, remapped_ros_topic))

        launcher.node(
            package=f"{ros_gz}_image",
            executable="image_bridge",
            name=f"{name}_image",
            remaps={"remappings": remappings},
        )


def spawn_gazebo_world_tf(world_frame=None):
    """
    Runs a static_transform_publisher to connect the ROS 'world' frame and a Gazebo world frame,
    if the Gazebo world frame is specified and different from 'world'.

    Args:
    - launcher: Instance of BetterLaunch for managing launching processes.
    - world_frame: Optional; the name of the Gazebo world frame. If None, retrieves the default from GazeboBridge.world().
    """
    launcher = BetterLaunch.instance()
    if world_frame is None:
        world_frame = GazeboBridge.world()

    if world_frame != "world":
        launcher.node(package= "tf2_ros",
            executable= "static_transform_publisher",
            name = "gazebo_world_tf",
            parameters = [{"frame_id": "world", "child_frame_id": world_frame}],)


def gazebo_launch(world_file, gz_args=None, full_world=None, save_after=5.0):
    """
    Args:
    - launcher: Instance of BetterLaunch to manage launching processes.
    - world_file: Path to the primary world file for the simulation.
    - gz_args: Optional arguments to pass to the Gazebo simulator.
    - full_world: If provided, path where the resulting SDF should be saved.
    - save_after: Time in seconds after which the world SDF should be saved.

    Returns:
    - An object or reference from the launcher.include call, typically used for further management.
    """
    launcher = BetterLaunch.instance()
    if isinstance(full_world, str) and os.path.exists(full_world):
        launch_file, launch_arguments = gazebo_launch_setup(full_world, gz_args)
    else:
        launch_file, launch_arguments = gazebo_launch_setup(world_file, gz_args)
        if isinstance(full_world, str):
            save_gazebo_world(launcher, full_world, save_after)

    return launcher.include(pkg=None, launch_file=launch_file)


def save_gazebo_world(dst, after=5.0):
    """
    Saves the current world under dst.
    Resolves any spawned URDF through their description parameter and converts to SDF.
    Uses a thread to delay the action of saving the world.
    """
    launcher = BetterLaunch.instance()
    def delayed_launch():
        time.sleep(after)
        launcher.node(
            package="better_launch",
            executable="generate_gz_world",
            params={"output_path": dst}
        )

    threading.Thread(target=delayed_launch).start()


def spawn_gazebo_model(
    name,
    topic="robot_description",
    model_file=None,
    spawn_args=None,
):
    """
    Spawns a model into Gazebo under the given name, from the given topic or file.
    The spawn_args dictionary can include additional options such as the initial pose.
    """
    launcher = BetterLaunch.instance()
    if spawn_args is None:
        spawn_args = {}

    spawn_args["name"] = name  
    if model_file is not None:
        spawn_args["file"] = model_file  
    else:
        spawn_args["topic"] = topic
    pkg = "ros_ign_gazebo" if get_gazebo_prefix() == "ign" else "ros_gz_sim"
    launcher.node(package=pkg, executable="create", cmd_args=spawn_args)
