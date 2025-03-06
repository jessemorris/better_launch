"""Additional convenience methods that aren't general enough to be added to the main namespace."""

__all__ = [
    "rviz",
    "read_robot_description",
    "joint_state_publisher",
    "robot_state_publisher",
]


import subprocess

from better_launch import BetterLaunch
from better_launch.elements import Node


def rviz(
    package: str = None,
    configfile: str = None,
    subdir: str = None,
    *,
    suppress_warnings: bool = False,
) -> Node:
    """Runs RViz with the given config file and optional warning level suppression.

    Parameters
    ----------
    package : str, optional
        Path to locate the config file in (if one is specified).
    config_file : str, optional
        Path to the RViz configuration file which will be resolved by :py:meth:`BetterLaunch.find`. Otherwise RViz will run with the default config.
    subdir : str, optional
        A path fragment the config file must be located in.
    suppress_warnings : bool, optional
        Whether to suppress warnings.

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

    if not suppress_warnings:
        args += ["--ros-args", "--log-level", "FATAL"]

    return bl.node("rviz2", "rviz2", "rviz2", anonymous=True, cmd_args=args)


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
    """
    bl = BetterLaunch.instance()

    filepath = bl.find(package, urdf_or_xacro, subdir)

    if filepath.endswith("urdf") and xacro_args is None:
        with open(filepath) as f:
            return f.read()

    cmd = ["xacro", filepath] + xacro_args

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


def joint_state_publisher(use_gui: bool, node_name: str = None, **kwargs) -> Node:
    """Starts a `joint_state_publisher` or `joint_state_publisher_gui`.

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
