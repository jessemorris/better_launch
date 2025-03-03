"""This module contains additional convenience functions for working with Gazebo. It takes inspiration from simple_launch.
"""

from typing import Literal, get_args
import os
from subprocess import check_output, STDOUT
from tempfile import NamedTemporaryFile

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

from better_launch import BetterLaunch
from better_launch.elements import Node


gazebo_launched_worlds = {}


def get_gazebo_version() -> str:
    """Get the Gazebo version from the environment variables.

    Returns
    -------
    str
        A Gazebo version name, e.g. 'garden'.
    """
    for var in ["IGN_VERSION", "IGNITION_VERSION", "GZ_VERSION"]:
        version = os.environ.get(var, None)
        if version:
            return "ign" if version == "fortress" else "gz"

    return None


def get_gazebo_prefix() -> str:
    """For a short time, Gazebo rebranded to "Ignition" before returning to Gazebo again. This function returns the associated prefix.

    Returns
    -------
    str
        The prefix indicating the type of Gazebo: "ign" for Ignition Gazebo or "gz" for Gazebo Classic.
    """

    version = get_gazebo_version()
    if version:
        return "ign" if version == "fortress" else "gz"

    try:
        get_package_share_directory("ros_gz")
        return "gz"
    except PackageNotFoundError:
        return "ign"


def declare_gazebo_axes(default_axes: dict[str, float] = None) -> dict[str, float]:
    """Declares classical Gazebo axes for e.g. defining the orientation of a model you're spawning.

    If `axes` is empty or None, all six axes (`x`, `y`, `z`, `yaw`, `pitch`, `roll`) are declared with a default value of `0.0`. Otherwise, only the specified axes are declared with the given defaults.

    Parameters
    ----------
    default_axes : dict[str, float], optional
        A dictionary specifying which axes to declare and their default values. If not provided, all six axes will be declared with a default of `0.0`.

    Returns
    -------
    dict[str, float]
        A dictionary where the keys are axis names (`x`, `y`, `z`, `yaw`, `pitch`, `roll`) and the values are their corresponding default values.
    """
    gz_axes = ["x", "y", "z", "yaw", "pitch", "roll"]

    if default_axes:
        axes = {ax: default_axes[ax] for ax in gz_axes if ax in default_axes}
    else:
        axes = {ax: 0.0 for ax in gz_axes}

    return axes


def get_gazebo_axes_args(declared_axes: dict[str, float] = None) -> list[str]:
    """Constructs a list of command-line arguments that can be used e.g. when spawning a new model.

    If `declared_axes` is None, the function defaults to declaring all six classical Gazebo axes (`x`, `y`, `z`, `yaw`, `pitch`, `roll`). See :py:meth:`get_gazebo_axes` for details.

    Parameters
    ----------
    declared_axes : dict[str, float], optional
        A dictionary specifying which Gazebo axes to include and their corresponding values. If None, all six axes will be initialized with a default value of `0.0`.

    Returns
    -------
    list[str]
        A list of command-line arguments formatted as `--axis value` pairs, suitable for passing to a Gazebo launch command.
    """
    if declared_axes is None:
        declared_axes = declare_gazebo_axes()

    return [f"--{axis} {value}" for axis, value in declared_axes.items()]


def start_gazebo_topic_bridge(
    *bridges: "GazeboBridge", node_name: str = "gz_bridge"
) -> tuple[Node, Node]:
    """Creates a `ros_gz_bridge::parameter_bridge` node with the specified `GazeboBridge` instances.

    This function sets up ROS-Gazebo bridges for different topic types. Standard topic bridges (non-image) are configured via a YAML parameter file, while image topics are handled separately through remappings.

    Parameters
    ----------
    *bridges : GazeboBridge
        One or more `GazeboBridge` instances defining the Gazebo-ROS topic bridges.
    node_name : str, optional
        Name of the bridge node, by default `"gz_bridge"`.

    Returns
    -------
    tuple[Node, Node]
        The up to two nodes that will be started by this function. The first node will be the node handling all regular topic bridges, the second the one for the image topics. Either of these may be `None`.
    """
    if len(bridges) == 0:
        return (None, None)

    bl = BetterLaunch.instance()

    ros_gz = "ros_" + get_gazebo_prefix()
    image_bridges = [bridge for bridge in bridges if bridge.is_image]
    std_config = sum((bridge.yaml() for bridge in bridges if not bridge.is_image), [])

    node_reg = None
    node_img = None

    if std_config:
        dst = NamedTemporaryFile().name
        with open(dst, "w") as file:
            file.write(std_config)

        node_reg = bl.node(
            f"{ros_gz}_bridge",
            "parameter_bridge",
            node_name,
            params={"config_file": dst},
        )

    if image_bridges:
        img_remaps = {}
        for bridge in image_bridges:
            for ext in ("", "/compressed", "/compressedDepth", "/theora"):
                remapped_gz_topic = f"{bridge.gz_topic}{ext}"
                remapped_ros_topic = f"{bridge.ros_topic}{ext}"
                img_remaps[remapped_gz_topic] = remapped_ros_topic

        node_img = bl.node(
            f"{ros_gz}_image",
            "image_bridge",
            f"{node_name}_image",
            remaps=img_remaps,
        )

    return (node_reg, node_img)


def spawn_gazebo_world_transform(gazebo_world_frame: str = None) -> Node:
    """
    Runs a static_transform_publisher to connect the ROS `world` frame and a Gazebo world frame, if the Gazebo world frame is specified and different from 'world'.

    Parameters
    ----------
    gazebo_world_frame : str, optional
        The name of the Gazebo world frame. If None, retrieves the default from :py:meth:`GazeboBridge.world`.

    Returns
    -------
    Node
        The spawned node instance.
    """
    if gazebo_world_frame is None:
        gazebo_world_frame = GazeboBridge.world()

    if gazebo_world_frame != "world":
        bl = BetterLaunch.instance()
        return bl.node(
            "tf2_ros",
            "static_transform_publisher",
            "gazebo_world_tf",
            params={"frame_id": "world", "child_frame_id": gazebo_world_frame},
        )

    return None


def gazebo_launch(
    world_file: str,
    gz_args: list[str] = None,
    world_save_file: str = None,
    save_after: float = 10.0,
) -> None:
    """Starts a Gazebo simulation with the specified world. If `world_save_file` is specified, the world will be saved as an SDF to that file after `save_after` seconds, including base world and spawned entities.

    Parameters
    ----------
    world_file : str
        Path to the primary world file for the simulation.
    gz_args : _type_, optional
        Optional arguments to pass to the Gazebo simulator.
    world_save_file : _type_, optional
        Path where the resulting SDF should be saved.
    save_after : float, optional
        Time in seconds after which the world SDF should be saved.
    """
    bl = BetterLaunch.instance()

    _, launch_file, launch_arguments = _get_gazebo_launch_args(world_file, gz_args)
    bl.include(launch_file, **launch_arguments)

    if world_save_file:
        gazebo_save_world(world_save_file, save_after)


def gazebo_save_world(filepath: str, after: float = 5.0):
    """Saves the current gazebo world to a file. Resolves any spawned URDF through their description parameter and converts to SDF.

    Parameters
    ----------
    filepath : str
        Path to the file to save the gazebo world to.
    after : float, optional
        How long to wait before saving.
    """
    bl = BetterLaunch.instance()

    def save():
        bl.node(
            package="better_launch",
            executable="generate_gz_world",
            params={"output_path": filepath},
        )

    if after > 0.0:
        bl.run_later(after, save)
    else:
        save()


def gazebo_spawn_model(
    model_name: str,
    topic: str = "robot_description",
    model_file: str = None,
    spawn_args: list[str] = None,
) -> Node:
    """
    Spawns a model into Gazebo under the given name from the specified topic or file.

    The spawn_args dictionary can include additional options, such as the initial pose of the model.

    Parameters
    ----------
    name : str
        The name of the model to spawn in the Gazebo environment.
    topic : str, optional
        The topic to retrieve the model description from.
    model_file : str, optional
        The path to the model file to spawn. If not provided, the model will be retrieved from the specified topic.
    spawn_args : list[str], optional
        Additional arguments for spawning the model, such as pose and other options.

    Returns
    -------
    Node
        The spawned node instance.
    """
    bl = BetterLaunch.instance()

    if spawn_args is None:
        spawn_args = []

    spawn_args += ["-name", model_name]
    if model_file is not None:
        spawn_args += ["-file", model_file]
    else:
        spawn_args += ["-topic", topic]

    pkg = "ros_ign_gazebo" if get_gazebo_prefix() == "ign" else "ros_gz_sim"
    return bl.node(
        pkg, "create", f"spawn_{model_name}", anonymous=True, cmd_args=spawn_args
    )


def _get_gazebo_launch_args(
    world_file: str,
    gz_args: list[str] = None,
) -> tuple[str, str, dict]:
    """Prepares the launch file and arguments for starting a Gazebo simulation based on the provided world file.

    Parameters
    ----------
    world_file : str
        The path to the world file to be loaded in the Gazebo simulator.
    gz_args : list[str], optional
        Optional additional arguments to pass to Gazebo, such as simulation parameters.

    Raises
    ------
    ValueError
        If the world_file cannot be found.

    Returns
    -------
    tuple[str, str, dict]
        - The name of the world foudn in the `world_file`.
        - The path to the launch file for Gazebo.
        - A dictionary of arguments to be passed to the Gazebo launch file.
    """
    bl = BetterLaunch.instance()

    world_file = bl.find(world_file)
    if not os.path.exists(world_file):
        raise ValueError("Could not find world file")

    full_args = [world_file]
    if gz_args:
        full_args += gz_args

    if get_gazebo_prefix() == "gz":
        launch_file = bl.find("ros_gz_sim", "gz_sim.launch.py")
        launch_arguments = {"gz_args": full_args}
    else:
        launch_file = bl.find("ros_ign_gazebo", "ign_gazebo.launch.py")
        launch_arguments = {"ign_args": full_args}

    if world_file in gazebo_launched_worlds:
        world_name = gazebo_launched_worlds[world_file]
    else:
        world_name = None
        with open(world_file) as f:
            while True:
                line = f.readline()

                if not line:
                    bl.logger.warning(
                        f"Could not get the name of the Gazebo world {world_file}"
                    )
                    break

                if "world name" in line:
                    world_name = line.split('"')[1]
                    gazebo_launched_worlds[world_file] = world_name
                    GazeboBridge.set_world_name(world_name)  # TODO
                    bl.logger.info(f"Found Gazebo world '{world_name}' in {world_file}")
                    break

    return world_name, launch_file, launch_arguments


class GazeboBridge:
    Direction = Literal["gz2ros", "ros2gz", "bidirectional"]

    _active_world_name = None
    _gz_exec = None

    # ros <-> gz mapping
    # from https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
    #
    # def generate_msg_map(readme) -> None:
    #     out = ["    msg_map = {"]
    #
    #     for line in readme.splitlines():
    #         if "|" in line and "/msg/" in line and "gz.msgs" in line:
    #             _, ros, gz, _ = line.split("|")
    #             out.append(f"    '{ros.strip()}': '{gz.strip()}',")
    #     print("\n".join(out)[:-1] + "}")

    msg_map = {
        "actuator_msgs/msg/Actuators": "gz.msgs.Actuators",
        "builtin_interfaces/msg/Time": "gz.msgs.Time",
        "geometry_msgs/msg/Point": "gz.msgs.Vector3d",
        "geometry_msgs/msg/Pose": "gz.msgs.Pose",
        "geometry_msgs/msg/PoseArray": "gz.msgs.Pose_V",
        "geometry_msgs/msg/PoseStamped": "gz.msgs.Pose",
        "geometry_msgs/msg/PoseWithCovariance": "gz.msgs.PoseWithCovariance",
        "geometry_msgs/msg/PoseWithCovarianceStamped": "gz.msgs.PoseWithCovariance",
        "geometry_msgs/msg/Quaternion": "gz.msgs.Quaternion",
        "geometry_msgs/msg/Transform": "gz.msgs.Pose",
        "geometry_msgs/msg/TransformStamped": "gz.msgs.Pose",
        "geometry_msgs/msg/Twist": "gz.msgs.Twist",
        "geometry_msgs/msg/TwistStamped": "gz.msgs.Twist",
        "geometry_msgs/msg/TwistWithCovariance": "gz.msgs.TwistWithCovariance",
        "geometry_msgs/msg/TwistWithCovarianceStamped": "gz.msgs.TwistWithCovariance",
        "geometry_msgs/msg/Vector3": "gz.msgs.Vector3d",
        "geometry_msgs/msg/Wrench": "gz.msgs.Wrench",
        "geometry_msgs/msg/WrenchStamped": "gz.msgs.Wrench",
        "gps_msgs/msg/GPSFix": "gz.msgs.NavSat",
        "nav_msgs/msg/Odometry": "gz.msgs.OdometryWithCovariance",
        "rcl_interfaces/msg/ParameterValue": "gz.msgs.Any",
        "ros_gz_interfaces/msg/Altimeter": "gz.msgs.Altimeter",
        "ros_gz_interfaces/msg/Contact": "gz.msgs.Contact",
        "ros_gz_interfaces/msg/Contacts": "gz.msgs.Contacts",
        "ros_gz_interfaces/msg/Dataframe": "gz.msgs.Dataframe",
        "ros_gz_interfaces/msg/Entity": "gz.msgs.Entity",
        "ros_gz_interfaces/msg/Float32Array": "gz.msgs.Float_V",
        "ros_gz_interfaces/msg/GuiCamera": "gz.msgs.GUICamera",
        "ros_gz_interfaces/msg/JointWrench": "gz.msgs.JointWrench",
        "ros_gz_interfaces/msg/Light": "gz.msgs.Light",
        "ros_gz_interfaces/msg/ParamVec": "gz.msgs.Param",
        "ros_gz_interfaces/msg/SensorNoise": "gz.msgs.SensorNoise",
        "ros_gz_interfaces/msg/StringVec": "gz.msgs.StringMsg_V",
        "ros_gz_interfaces/msg/TrackVisual": "gz.msgs.TrackVisual",
        "ros_gz_interfaces/msg/VideoRecord": "gz.msgs.VideoRecord",
        "rosgraph_msgs/msg/Clock": "gz.msgs.Clock",
        "sensor_msgs/msg/BatteryState": "gz.msgs.BatteryState",
        "sensor_msgs/msg/CameraInfo": "gz.msgs.CameraInfo",
        "sensor_msgs/msg/FluidPressure": "gz.msgs.FluidPressure",
        "sensor_msgs/msg/Image": "gz.msgs.Image",
        "sensor_msgs/msg/Imu": "gz.msgs.IMU",
        "sensor_msgs/msg/JointState": "gz.msgs.Model",
        "sensor_msgs/msg/Joy": "gz.msgs.Joy",
        "sensor_msgs/msg/LaserScan": "gz.msgs.LaserScan",
        "sensor_msgs/msg/MagneticField": "gz.msgs.Magnetometer",
        "sensor_msgs/msg/NavSatFix": "gz.msgs.NavSat",
        "sensor_msgs/msg/PointCloud2": "gz.msgs.PointCloudPacked",
        "std_msgs/msg/Bool": "gz.msgs.Boolean",
        "std_msgs/msg/ColorRGBA": "gz.msgs.Color",
        "std_msgs/msg/Empty": "gz.msgs.Empty",
        "std_msgs/msg/Float32": "gz.msgs.Float",
        "std_msgs/msg/Float64": "gz.msgs.Double",
        "std_msgs/msg/Header": "gz.msgs.Header",
        "std_msgs/msg/Int32": "gz.msgs.Int32",
        "std_msgs/msg/String": "gz.msgs.StringMsg",
        "std_msgs/msg/UInt32": "gz.msgs.UInt32",
        "tf2_msgs/msg/TFMessage": "gz.msgs.Pose_V",
        "trajectory_msgs/msg/JointTrajectory": "gz.msgs.JointTrajectory",
        "vision_msgs/msg/Detection2D": "gz.msgs.AnnotatedAxisAligned2DBox",
        "vision_msgs/msg/Detection2DArray": "gz.msgs.AnnotatedAxisAligned2DBox_V",
    }

    def __init__(
        self,
        gz_topic: str,
        ros_topic: str,
        ros_msg: str,
        direction: Direction,
        gz_msg: str = None,
    ) -> None:
        """
        Initializes a ROS-to-Gazebo bridge with the specified parameters.

        This constructor sets up the bridge between a ROS topic and a Gazebo topic. It maps the ROS message type to a Gazebo message type and validates the direction of communication (i.e. ROS to Gazebo, Gazebo to ROS, or bidirectional).

        Parameters
        ----------
        gz_topic : str
            The Gazebo topic to be bridged.
        ros_topic : str
            The ROS topic to be bridged.
        ros_msg : str
            The message type used on the ROS topic.
        direction : Direction
            The direction of the bridge.
        gz_msg : str, optional
            The Gazebo message type. If not provided, it will be inferred from the ROS message type.

        Raises
        ------
        ValueError
            If the provided message type or direction is invalid.
        """

        if "/msg/" not in ros_msg:
            ros_msg = ros_msg.replace("/", "/msg/")

        if gz_msg is not None:
            self.gz_msg = gz_msg
        else:
            if ros_msg not in self.msg_map:
                raise ValueError(f"Unknown message type '{ros_msg}', try giving an explicit gz_msg")
        
            self.gz_msg = self.msg_map[ros_msg]

        if direction not in get_args(self.Direction):
            raise ValueError(f"Invalid bridge direction '{direction}'")

        self.gz_topic = gz_topic
        self.ros_topic = ros_topic

        self.is_image = (ros_msg == "sensor_msgs/msg/Image")
        self.direction = direction
        self.ros_msg = ros_msg

    @classmethod
    def gz_exec(cls) -> str:
        if not cls._gz_exec:
            cls._gz_exec = get_gazebo_prefix()
        return cls._gz_exec

    def yaml(self) -> None:
        """Returns a YAML snippet that can be used to configure other bridges.
        """
        gz_exec = self.gz_exec()

        dir_string = "BIDIRECTIONAL"
        if self.direction == "gz2ros":
            dir_string = f"{gz_exec.upper()}_TO_ROS"
        elif self.direction == "ros2gz":
            dir_string = f"ROS_TO_{gz_exec.upper()}"

        if gz_exec == "ign":
            self.gz_msg = self.gz_msg.replace("gz.", "ignition.")

        return f"""\
- {gz_exec}_topic_name: {self.gz_topic}
  {gz_exec}_type_name: {self.gz_msg}
  ros_topic_name: {self.ros_topic}
  ros_type_name: {self.ros_msg}
  direction: {dir_string}
"""

    @classmethod
    def set_world_name(cls, name: str) -> None:
        """
        Overwrite world name in order to avoid communicating with Gazebo
        Useful when Gazebo is launched in an included file, where the world name cannot be guessed
        """
        GazeboBridge._active_world_name = name

    @classmethod
    def world(cls) -> str:
        """Return the name of the currently loaded world. If it was not loaded from better_launch, Gazebo will be asked directly.

        Returns
        -------
        str
            The name of a Gazebo world.

        Raises
        ------
        ValueError
            If querying Gazebo for the world name failed.
        """
        if GazeboBridge._active_world_name is not None:
            return GazeboBridge._active_world_name

        try:
            cmd = [cls.gz_exec(), "model", "--list"]
            output = check_output(cmd, stderr=STDOUT).decode()
        except Exception as e:
            raise ValueError("Failed to list loaded Gazebo models: " + e)

        if not output or "timed out" in output:
            raise ValueError("No loaded Gazebo models or request timed out")

        world_name = output.replace("]", "[").split("[")[1]

        if not world_name:
            raise ValueError(f"The command {' '.join(cmd)} did not return a parseable world name (output was '{output}')")

        GazeboBridge._active_world_name = world_name
        return world_name

    @classmethod
    def get_model_path(cls, model: str) -> str:
        """
        Constructs a Gazebo model prefix string for the given model name.

        Parameters
        ----------
        model : str
            The name of the model.

        Returns
        -------
        str
            A string representing the model's prefix in the Gazebo world.
        """
        return f"/world/{cls.world()}/model/{model}"

    @classmethod
    def get_model_topic(cls, model: str, topic: str) -> str:
        """
        Constructs a tuple representing the model topic in Gazebo.

        Parameters
        ----------
        model : str
            The model name.
        topic : str
            The topic name related to the model.

        Returns
        -------
        str
            The path of the given model's specified topic.
        """
        # NOTE simple_launch only returns f"/model/{model}/{topic}", but that seems wrong
        return f"{cls.get_model_path(model)}/{topic}"

    @classmethod
    def make_clock_bridge(cls) -> "GazeboBridge":
        """
        Creates a GazeboBridge instance for the /clock topic.

        Returns
        -------
        GazeboBridge
            An instance of the GazeboBridge for the clock topic.
        """
        return GazeboBridge(
            "/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz2ros"
        )

    @classmethod
    def make_(cls, model: str) -> "GazeboBridge":
        """
        Creates a GazeboBridge instance for the joint states of a given model.

        Parameters
        ----------
        model : str
            The model name to associate with the joint state topic.

        Returns
        -------
        GazeboBridge
            An instance of the GazeboBridge for the joint state topic.
        """
        js_gz_topic = f"/world/{GazeboBridge.world()}/model/{model}/joint_state"
        return GazeboBridge(
            js_gz_topic, "joint_states", "sensor_msgs/JointState", "gz2ros"
        )
