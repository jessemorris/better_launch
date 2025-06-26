"""Additional convenience methods that aren't general enough to be added to the main namespace."""

__all__ = [
    "rviz",
    "read_robot_description",
    "joint_state_publisher",
    "robot_state_publisher",
    "static_transform_publisher",
    "run_command",
]


from typing import Sequence
import subprocess

from better_launch import BetterLaunch
from better_launch.elements import Node


def rviz(
    package: str = None,
    configfile: str = None,
    subdir: str = None,
    *,
    extra_args: list[str] = None,
) -> Node:
    """Runs RViz2 with the given config file and optional warning level suppression.

    Parameters
    ----------
    package : str, optional
        Path to locate the config file in (if one is specified).
    config_file : str, optional
        Path to the RViz2 configuration file which will be resolved by :py:meth:`BetterLaunch.find`. Otherwise RViz2 will run with the default config.
    subdir : str, optional
        A path fragment the config file must be located in.
    extra_args : list[str], optional
        Additional args to pass to the RViz2 executable.

    Returns
    -------
    Node
        The spawned node instance.
    """
    bl = BetterLaunch.instance()

    args = []
    if configfile:
        configfile = bl.find(package, configfile, subdir)
        args += ["-d", configfile]

    if extra_args:
        args.extend(extra_args)

    # rviz2 doesn't support the --log-level argument nodes usually accept
    return bl.node(
        "rviz2", "rviz2", "rviz2", anonymous=True, cmd_args=args, log_level=None
    )


def read_robot_description(
    package: str = None,
    urdf_or_xacro: str = None,
    subdir: str = None,
    *,
    xacro_args: list[str] = None,
) -> str | None:
    """Returns the contents of a robot description after a potential xacro parse.

    The file is resolved using :py:meth:`BetterLaunch.find`. If the description file ends with `.urdf` and `xacro_args` is not provided, it reads the URDF file directly. Otherwise it runs `xacro` to generate the URDF from a `.xacro` file.

    Parameters
    ----------
    package : str, optional
        The package where the robot description file is located. May be `None` (see :py:meth:`BetterLaunch.find`)
    urdf_or_xacro : str, optional
        The name of the robot description file (URDF or XACRO).
    subdir : str, optional
        A path fragment the description file must be located in.
    xacro_args : list of str, optional
        Additional arguments to pass to `xacro` when processing `.xacro` files.

    Returns
    -------
    str | None
        The parsed URDF XML as a string if successful, `None` otherwise.

    Raises
    ------
    ValueError
        If the xacro command encountered an error while processing the file.
    """
    bl = BetterLaunch.instance()

    filepath = bl.find(package, urdf_or_xacro, subdir)

    if filepath.endswith("urdf") and xacro_args is None:
        with open(filepath) as f:
            return f.read()

    args = [filepath]
    if xacro_args:
        args.extend(xacro_args)

    try:
        return run_command("xacro", args)
    except subprocess.CalledProcessError as e:
        raise ValueError(f"Xacro failed ({e.returncode}): {e.output}") from e


def joint_state_publisher(use_gui: bool, node_name: str = None, **kwargs) -> Node:
    """Starts a `joint_state_publisher` or `joint_state_publisher_gui` node.

    Parameters
    ----------
    use_gui : bool
        Whether to use the GUI version of the `joint_state_publisher`.
    node_name : str, optional
        The name of the node. If not provided the name of the executable will be used. Will be anonymized unless `anonymous=False` is passed.
    **kwargs : dict, optional
        Additional arguments to pass to the node (e.g. name, remaps, params, etc.). See :py:meth:`BetterLaunch.node`.

    Returns
    -------
    Node
        The spawned node instance.
    """
    bl = BetterLaunch.instance()

    kwargs.setdefault("anonymous", True)

    if use_gui:
        return bl.node(
            "joint_state_publisher_gui",
            "joint_state_publisher_gui",
            node_name or "joint_state_publisher_gui",
            **kwargs,
        )
    else:
        return bl.node(
            "joint_state_publisher",
            "joint_state_publisher",
            node_name or "joint_state_publisher",
            **kwargs,
        )


def robot_state_publisher(
    package: str = None,
    urdf_or_xacro: str = None,
    subdir: str = None,
    *,
    xacro_args: list[str] = None,
    node_name: str = None,
    **kwargs,
) -> Node:
    """Start a Robot State Publisher node using the given URDF/Xacro file. The file is resolved using :py:meth:`BetterLaunch.find`.

    Parameters
    ----------
    package : str, optional
        The name of the package containing the robot description file.
    urdf_or_xacro : str, optional
        The name of the URDF or Xacro file describing the robot model.
    subdir : str, optional
        A path fragment the description file must be located in.
    xacro_args : list of str, optional
        Additional arguments to pass to the Xacro processor when processing `.xacro` files.
    node_name : str, optional
        The name of the node. If not provided the name of the executable will be used. Will be anonymized unless `anonymous=False` is passed.
    **kwargs : dict, optional
        Additional arguments for the node, such as remappings or parameters.

    Returns
    -------
    Node
        The spawned node instance.
    """
    bl = BetterLaunch.instance()

    urdf_xml = read_robot_description(
        package,
        urdf_or_xacro,
        subdir,
        xacro_args=xacro_args,
    )

    kwargs.setdefault("anonymous", True)
    params = kwargs.pop("params", {})
    params["robot_description"] = urdf_xml

    return bl.node(
        "robot_state_publisher",
        "robot_state_publisher",
        node_name,
        params=params,
        **kwargs,
    )


def static_transform_publisher(
    parent_frame: str,
    child_frame: str,
    pos: Sequence[float] = None,
    rot: Sequence[float] = None,
) -> Node:
    """Publish a static transform between two frames.

    Parameters
    ----------
    parent_frame : str
        The parent or source frame.
    child_frame : str
        The child or target frame.
    pos : Sequence[float], optional
        The xyz-translation from parent to child frame. You may also pass a sequence of 6 or 7 floats in order to specify a full pose. In this case, `rot` will be ignored.
    rot : Sequence[float], optional
        The rotation between the parent and child frame. If length is 3, the values are interpreted as roll-pitch-yaw euler angles. If length is 4, an xyzw-quaternion is assumed.

    Returns
    -------
    Node
        The node running the publisher process.

    Raises
    ------
    ValueError
        If pos or rot have the wrong length.
    """
    args = ["--frame-id", parent_frame, "--child-frame-id", child_frame]

    # Position may also hold the pose
    if pos is not None:
        args.extend(["--x", pos[0], "--y", pos[1], "--z", pos[2]])
        if len(pos) == 3:
            pass
        elif len(pos) == 4 and pos[3] == 1.0:
            # Homogenous translation vector, it's fine
            pass
        elif len(pos) in (6, 7):
            rot = pos[3:]
        else:
            raise ValueError("Position has dubious length %d", len(pos))

    if rot is not None:
        if len(rot) == 3:
            args.extend(["--roll", rot[0], "--pitch", rot[1], "--yaw", rot[2]])
        elif len(rot) == 4:
            args.extend(
                ["--qx", rot[0], "--qy", rot[1], "--qz", rot[2], "--qw", rot[3]]
            )
        else:
            raise ValueError("Rotation has dubious length %d", len(rot))

    bl = BetterLaunch.instance()
    return bl.node(
        "tf2_ros",
        "static_transform_publisher",
        "gazebo_world_tf",
        cmd_args=args,
        raw=True,
    )


def run_command(cmd: str, args: str | list[str] = None) -> str:
    """Run the specified command and return its output. 

    Parameters
    ----------
    cmd : str
        The command to run. Can either be an absolute path to an executable or any file found on `PATH`. 
    args : str | list[str], optional
        Additional arguments to pass to the command. If a string is passed it will be split on spaces. Pass a list instead to have more control over which arguments to treat as one.

    Returns
    -------
    str
        The output of the command without the trailing newline.

    Raises
    ------
    subprocess.CalledProcessError
        If the command had a non-zero exit code. See the raised error's `returncode` and `output` attributes for details.
    """
    run = [cmd]
    if args:
        if isinstance(args, str):
            args = args.split(" ")
        run.extend(args)

    return subprocess.check_output(run, stderr=subprocess.STDOUT).decode().rstrip("\n")


def spawn_controller(controller: str, manager: str = "controller_manager") -> Node:
    """Spawn the specified controller. 

    Parameters
    ----------
    controller : str
        The controller to spwawn.
    manager : str
        The name of the controller_manager node.

    Returns
    -------
    Node
        The node running the spawner process.
    """
    bl = BetterLaunch.instance()
    return bl.node(
        package="controller_manager",
        executable="spawner",
        cmd_args=[controller, "--controller-manager", manager],
        raw=True,
    )
