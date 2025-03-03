"""This module contains additional convenience functions that would have bloated the main API. It takes inspiration from simple_launch"""

import os
import threading
import time
import subprocess
from tempfile import NamedTemporaryFile

from better_launch import BetterLaunch
from better_launch.utils.gazebo import GazeboBridge, get_gazebo_prefix, gazebo_launch_setup
from better_launch.ros import logging as roslog


def joint_state_publisher(use_gui: bool, node_name: str = None, **kwargs) -> None:
    """
    Adds a joint_state_publisher or joint_state_publisher_gui with passed arguments as parameters.

    Parameters
    ----------
    use_gui : bool
        Whether to use the GUI version of the joint_state_publisher.
    node_name : str, optional
        The name of the node. If not provided the name of the executable will be used. Will be anonymized unless `anonymous=False` is passed.
    **kwargs : dict, optional
        Additional arguments to pass to the node (e.g. name, remaps, params, etc.). See :py:meth:`BetterLaunch.node`.
    """
    launcher = BetterLaunch.instance()

    kwargs.setdefault("anonymous", True)

    if use_gui:
        launcher.node(
            "joint_state_publisher_gui",
            "joint_state_publisher_gui",
            node_name or "joint_state_publisher_gui",
            **kwargs,
        )
    else:
        launcher.node(
            "joint_state_publisher",
            "joint_state_publisher",
            node_name or "joint_state_publisher",
            **kwargs,
        )


def rviz(config_file: str = None, suppress_warnings: bool = False) -> None:
    """
    Runs RViz with the given config file and optional warning level suppression.

    Parameters
    ----------
    config_file : str, optional
        Path to the RViz configuration file which will be resolved by :py:meth:`BetterLaunch.find`. Otherwise RViz will run with the default config.
    suppress_warnings : bool, optional
        Whether to suppress warnings.
    """
    launcher = BetterLaunch.instance()

    args = []
    if config_file:
        config_file = launcher.find(config_file)
        args += ["-d", config_file]
    
    if not suppress_warnings:
        args += ["--ros-args", "--log-level", "FATAL"]

    launcher.node("rviz2", "rviz2", "rviz2", anonymous=True, cmd_args=args)


def read_robot_description(
    package: str = None,
    urdf_or_xacro: str = None,
    xacro_args: list[str] = None,
) -> str | None:
    """
    Returns the robot description after a potential xacro parse. The file is resolved using :py:meth:`BetterLaunch.find`.

    If the description file ends with `.urdf` and `xacro_args` is not provided, it reads the URDF file directly. Otherwise it runs `xacro` to generate the URDF from a `.xacro` file.

    Parameters
    ----------
    package : str, optional
        The package where the robot description file is located.
    urdf_or_xacro : str, optional
        The name of the robot description file (URDF or XACRO).
    xacro_args : list of str, optional
        Additional arguments to pass to `xacro` when processing `.xacro` files.

    Returns
    -------
    str | None
        The parsed URDF XML as a string if successful, `None` otherwise.
    """
    bl = BetterLaunch.instance()

    filepath = bl.find(package, urdf_or_xacro)

    if filepath.endswith("urdf") and xacro_args is None:
        with open(filepath) as f:
            return f.read()

    cmd = ["xacro", filepath] + (
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
                bl.logger.warning(f"Error processing xacro: {stderr}")
                return None
    except Exception as e:
        bl.logger.warning(f"Failed to execute xacro command: {e}")
        return None


def robot_state_publisher(
    package: str = None,
    urdf_or_xacro: str = None,
    xacro_args: list[str] = None,
    node_name: str = None,
    **kwargs,
) -> None:
    """
    Start a Robot State Publisher node using the given URDF/Xacro file. The file is resolved using :py:meth:`BetterLaunch.find`.

    Parameters
    ----------
    package : str, optional
        The name of the package containing the robot description file. 
    urdf_or_xacro : str, optional
        The name of the URDF or Xacro file describing the robot model.
    xacro_args : list of str, optional
        Additional arguments to pass to the Xacro processor when processing `.xacro` files.
    node_name : str, optional
        The name of the node. If not provided the name of the executable will be used. Will be anonymized unless `anonymous=False` is passed.
    **kwargs : dict, optional
        Additional arguments for the node, such as remappings or parameters.
    """
    bl = BetterLaunch.instance()

    try:
        urdf_xml = read_robot_description(
            package,
            urdf_or_xacro,
            xacro_args=xacro_args,
        )

        kwargs.setdefault("anonymous", True)
        params = kwargs.pop("params", {})
        params["robot_description"] = urdf_xml

        bl.node(
            "robot_state_publisher",
            "robot_state_publisher",
            node_name or "robot_state_publisher",
            params=params,
            **kwargs,
        )
    except FileNotFoundError as e:
        bl.logger.warning(f"Error loading robot description: {e}")
    except Exception as e:
        bl.logger.warning(f"Error launching robot state publisher node: {e}")


def declare_gazebo_axes(default_axes: dict[str, float] = None) -> dict[str, float]:
    """
    Declares classical Gazebo axes as launch arguments.

    If `axes` is empty or None, it declares all six axes (`x`, `y`, `z`, `yaw`, `pitch`, `roll`)
    with a default value of `0.0`. Otherwise, it declares only the specified axes with the given defaults.

    Parameters
    ----------
    default_axes : dict[str, float], optional
        A dictionary specifying which axes to declare and their default values.
        If not provided, all six axes will be declared with a default of `0.0`.

    Returns
    -------
    dict[str, float]
        A dictionary where the keys are axis names (`x`, `y`, `z`, `yaw`, `pitch`, `roll`)
        and the values are their corresponding default values.
    """
    gz_axes = ["x", "y", "z", "yaw", "pitch", "roll"]

    if default_axes:
        axes = {ax: default_axes[ax] for ax in gz_axes if ax in default_axes}
    else:
        axes = {ax: 0.0 for ax in gz_axes}

    return axes


def gazebo_axes_args(declared_axes: dict[str, float] = None) -> list[str]:
    """
    Constructs a list of command-line arguments for a Gazebo simulation based on declared axes.

    If `declared_axes` is None, the function defaults to declaring all six classical Gazebo axes
    (`x`, `y`, `z`, `yaw`, `pitch`, `roll`) with a value of `0.0`. Otherwise, it constructs arguments
    using the provided axis-value pairs.

    Parameters
    ----------
    declared_axes : dict[str, float], optional
        A dictionary specifying which Gazebo axes to include and their corresponding values.
        If None, all six axes will be initialized with a default value of `0.0`.

    Returns
    -------
    list[str]
        A list of command-line arguments formatted as `--axis value` pairs,
        suitable for passing to a Gazebo launch command.
    """
    if declared_axes is None:
        declared_axes = declare_gazebo_axes()

    args = [f"--{axis} {value}" for axis, value in declared_axes.items()]

    return args


def create_gazebo_bridge(*bridges: GazeboBridge, name: str = "gz_bridge") -> None:
    """
    Creates a `ros_gz_bridge::parameter_bridge` node with the specified `GazeboBridge` instances.

    This function sets up ROS-Gazebo bridges for different topic types. Standard topic bridges
    (non-image) are configured via a YAML parameter file, while image topics are handled separately
    through remappings.

    Parameters
    ----------
    *bridges : GazeboBridge
        One or more `GazeboBridge` instances defining the Gazebo-ROS topic bridges.
    name : str, optional
        Name of the bridge node, by default `"gz_bridge"`.

    Returns
    -------
    None
        This function does not return a value but launches the necessary ROS-Gazebo bridge nodes.
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


def spawn_gazebo_world_tf(world_frame=None) -> None:
    """
    Runs a static_transform_publisher to connect the ROS 'world' frame and a Gazebo world frame,
    if the Gazebo world frame is specified and different from 'world'.

    Args:
    - launcher: Instance of BetterLaunch for managing launching processes.
    - world_frame: Optional; the name of the Gazebo world frame. If None, retrieves the default from GazeboBridge.world().

    Returns
    -------
    None
        This function does not return anything.
    """
    launcher = BetterLaunch.instance()
    if world_frame is None:
        world_frame = GazeboBridge.world()

    if world_frame != "world":
        launcher.node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gazebo_world_tf",
            parameters=[{"frame_id": "world", "child_frame_id": world_frame}],
        )


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
            save_gazebo_world(full_world, save_after)

    return launcher.include(pkg=None, launch_file=launch_file)


def save_gazebo_world(dst, after=5.0):
    """
    Saves the current world under dst.
    Resolves any spawned URDF through their description parameter and converts to SDF.
    Uses a thread to delay the action of saving the world.

    Returns
    -------
    None
        This function does not return anything.
    """
    launcher = BetterLaunch.instance()

    def delayed_launch():
        time.sleep(after)
        launcher.node(
            package="better_launch",
            executable="generate_gz_world",
            params={"output_path": dst},
        )

    threading.Thread(target=delayed_launch).start()


def spawn_gazebo_model(
    name,
    topic="robot_description",
    model_file=None,
    spawn_args=None,
) -> None:
    """
    Spawns a model into Gazebo under the given name, from the specified topic or file.

    The spawn_args dictionary can include additional options, such as the initial pose of the model.

    Parameters
    ----------
    name : str
        The name of the model to spawn in the Gazebo environment.
    topic : str, optional
        The topic to retrieve the model description from. Defaults to "robot_description".
    model_file : str, optional
        The path to the model file to spawn. If not provided, the model will be retrieved from the specified topic.
    spawn_args : dict[str, str], optional
        Additional arguments for spawning the model, such as pose and other options. Defaults to an empty dictionary.

    Returns
    -------
    None
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
