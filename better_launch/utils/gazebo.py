from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from os.path import join, exists
import logging
from os import environ
from shlex import split
from typing import Optional, Tuple
from better_launch import BetterLaunch

logger = logging.getLogger(__name__)


def _silent_exec(cmd) -> str:
    """
    Executes the given command and returns the output
    Returns an empty list if any error
    """
    from subprocess import check_output, STDOUT

    if isinstance(cmd, str):
        cmd = split(cmd)
    try:
        return check_output(cmd, stderr=STDOUT).decode()
    except:
        return ""


gazebo_launched_worlds = []


def gazebo_launch_setup(
    world_file: str,
    gz_args: Optional[str] = None,
    show_args: bool = False,
    launcher: Optional[BetterLaunch] = None,
) -> Tuple[str, dict]:
    """
    Prepares the launch file and arguments for starting a Gazebo simulation based on the provided world file.

    Parameters
    ----------
    world_file : str
        The path to the world file to be loaded in the Gazebo simulator.
    gz_args : str, optional
        Optional additional arguments to pass to Gazebo, such as simulation parameters.
    show_args : bool, optional
        Whether to log and display world information, by default False.
    launcher : BetterLaunch, optional
        The launcher instance to manage logging and other functionality, by default None.

    Raises
    ------
    ValueError
        If the launcher is not provided.

    Returns
    -------
    Tuple[str, dict]
        - The path to the launch file for Gazebo (`str`).
        - A dictionary of arguments to be passed to the Gazebo launch file (`dict`).
    """
    if launcher is None:
        raise ValueError("Launcher must be provided")

    logger = launcher.get_logger()

    full_args = world_file + (" " + gz_args if gz_args else "")

    if get_gazebo_prefix() == "gz":
        launch_file = join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
        )
        launch_arguments = {"gz_args": full_args}
    else:
        launch_file = join(
            get_package_share_directory("ros_ign_gazebo"),
            "launch",
            "ign_gazebo.launch.py",
        )
        launch_arguments = {"ign_args": full_args}

    valid_file = world_file if exists(world_file) else None
    if not valid_file:
        for arg in full_args.split():
            if exists(arg):
                valid_file = arg
                break

    if valid_file:
        if show_args and valid_file not in gazebo_launched_worlds:
            gazebo_launched_worlds.append(valid_file)
            line = _silent_exec(f"grep 'world name' {valid_file}")
            world = line.split('"')[1] if line else None
            if world:
                logger.info(f'Gazebo world "{world}" found @ {valid_file}')
                GazeboBridge.set_world_name(world)
            else:
                logger.warning(f"Could not get the name of Gazebo world {valid_file}")
    else:
        logger.error(f'Could not read Gazebo world @ "{world_file}"')

    return launch_file, launch_arguments


def get_gazebo_prefix() -> str:
    """
    Returns
    -------
    str
        The prefix indicating the type of Gazebo: "ign" for Ignition Gazebo or "gz" for Gazebo Classic.
    """

    for env in ("IGN_VERSION", "GZ_VERSION"):
        if env in environ:
            return "ign" if environ[env] == "fortress" else "gz"

    try:
        get_package_share_directory("ros_gz")
        return "gz"
    except PackageNotFoundError:
        return "ign"


class GazeboBridge:
    logger = logging.getLogger("GazeboBridge")

    gz2ros = "["
    ros2gz = "]"
    bidirectional = "@"
    _world_name = None
    _gz_exec = None
    """

    ros <-> gz mapping
    from https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
    """

    def generate_msg_map(ros_gz_bridge_readme) -> None:
        """
        function to regenerate known messages from readme
        """
        out = ["    msg_map = {"]

        for line in ros_gz_bridge_readme.splitlines():
            if "|" in line and "/msg/" in line and "gz.msgs" in line:
                _, ros, gz, _ = line.split("|")
                out.append(f"    '{ros.strip()}': '{gz.strip()}',")
        print("\n".join(out)[:-1] + "}")

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

    def __init__(self, gz_topic, ros_topic, ros_msg, direction, gz_msg=None) -> None:
        """
        Initializes a ROS-to-Gazebo bridge with the specified parameters.

        This constructor sets up the bridge between a ROS topic and a Gazebo topic. It maps the ROS message type to a Gazebo message type
        and validates the direction of communication (e.g., ROS to Gazebo, Gazebo to ROS, or bidirectional). If the message type or
        direction is invalid, an error is logged.

        Parameters
        ----------
        gz_topic : str
            The Gazebo topic to be bridged.
        ros_topic : str
            The ROS topic to be bridged.
        ros_msg : str
            The message type used on the ROS topic.
        direction : str
            The direction of the bridge, either "gz2ros", "ros2gz", or "bidirectional".
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
        elif ros_msg not in self.msg_map:
            logger.error(
                f'Cannot build a ros <-> gz bridge for message "{ros_msg}": unknown type or give explicit gz_msg'
            )
            return
        else:
            self.gz_msg = self.msg_map[ros_msg]

        if not GazeboBridge.valid(direction):
            logger.error(
                f'Cannot build ros <-> gz bridge with direction "{direction}": use GazeboBrige.{{gz2ros,ros2gz,bidirectional}}'
            )
            return

        self.gz_topic = gz_topic
        self.ros_topic = ros_topic

        self.is_image = ros_msg == "sensor_msgs/msg/Image"
        self.direction = direction
        self.ros_msg = ros_msg

    def yaml(self) -> None:
        """
        use YAML-based config for other bridges
        - topic_name: "chatter"
            ign_topic_name: "ign_chatter"
            ros_type_name: "std_msgs/msg/String"
            ign_type_name: "ignition.msgs.StringMsg"
            direction: BIDIRECTIONAL  # Default "BIDIRECTIONAL" - Bridge both directions
                                    # "IGN_TO_ROS" - Bridge Ignition topic to ROS
                                    # "ROS_TO_IGN" - Bridge ROS topic
        """

        if GazeboBridge._gz_exec is None:
            GazeboBridge._gz_exec = get_gazebo_prefix()

        direction = "BIDIRECTIONAL"
        if self.direction == self.gz2ros:
            direction = f"{GazeboBridge._gz_exec.upper()}_TO_ROS"
        elif self.direction == self.ros2gz:
            direction = f"ROS_TO_{GazeboBridge._gz_exec.upper()}"

        if GazeboBridge._gz_exec == "ign":
            self.gz_msg = self.gz_msg.replace("gz.", "ignition.")

        return (
            f"- {GazeboBridge._gz_exec}_topic_name: ",
            self.gz_topic,
            "\n",
            f"  {GazeboBridge._gz_exec}_type_name: ",
            self.gz_msg,
            "\n",
            "  ros_topic_name: ",
            self.ros_topic,
            "\n",
            "  ros_type_name: ",
            self.ros_msg,
            "\n",
            "  direction: ",
            direction,
            "\n",
        )

    @staticmethod
    def valid(direction):
        return direction in (
            GazeboBridge.gz2ros,
            GazeboBridge.ros2gz,
            GazeboBridge.bidirectional,
        )

    @staticmethod
    def set_world_name(name: str) -> None:
        """
        Overwrite world name in order to avoid communicating with Gazebo
        Useful when Gazebo is launched in an included file, where the world name cannot be guessed
        """
        GazeboBridge._world_name = name

    @staticmethod
    def world(show_args=False) -> Optional[str]:
        if GazeboBridge._world_name is not None:
            return GazeboBridge._world_name

        if show_args:
            # set a dummy world model, we are not running anyway
            GazeboBridge._world_name = "dummy"  # silent future calls
            logger.warning(
                "This launch file will request information on a running Gazebo instance at the time of the launch"
            )
            return GazeboBridge._world_name

        candidates = ["gz", "ign"]
        for key in ("GZ_VERSION", "IGNITION_VERSION"):
            if key in environ:
                if environ[key] == "fortress":
                    candidates = ["ign"]
                else:
                    candidates = ["gz"]
                break

        for GazeboBridge._gz_exec in candidates:
            out = _silent_exec([GazeboBridge._gz_exec, "model", "--list"])
            if out == "" or "timed out" in out:
                continue
            GazeboBridge._world_name = out.replace("]", "[").split("[")[1]
            break

        if GazeboBridge._world_name is None:
            logger.error(
                f'GazeboBridge: could not find any Gazebo instance (tried {" and ".join(candidates)}), run Gazebo before this launch file.'
            )
            return

        logger.info(f'GazeboBridge: found world "{GazeboBridge._world_name}"')
        return GazeboBridge._world_name

    @staticmethod
    def model_prefix(model: str) -> str:
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
        if isinstance(model, str):
            return f"/world/{GazeboBridge.world()}/model/{model}"
        return f"/world/{GazeboBridge.world()}/model/", model

    @staticmethod
    def model_topic(model, topic):
        return ("/model/", model, "/", topic)

    @staticmethod
    def model_topic(model: str, topic: str) -> tuple:
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
        tuple
            A tuple representing the model and topic for Gazebo.
        """
        return ("/model/", model, "/", topic)

    @staticmethod
    def clock() -> "GazeboBridge":
        """
        Creates a GazeboBridge instance for the /clock topic.

        Returns
        -------
        GazeboBridge
            An instance of the GazeboBridge for the clock topic.
        """
        return GazeboBridge(
            "/clock", "/clock", "rosgraph_msgs/msg/Clock", GazeboBridge.gz2ros
        )

    @staticmethod
    def joint_states_bridge(model: str) -> "GazeboBridge":
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
            js_gz_topic, "joint_states", "sensor_msgs/JointState", GazeboBridge.gz2ros
        )
