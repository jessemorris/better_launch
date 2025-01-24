# TODO Create convenience module for common stuff like joint_state_publisher, robot_state_publisher, moveit, rviz, gazebo (see simple_launch)
from better_launch import BetterLaunch
import subprocess
from .gazebo import only_show_args, silent_exec, GazeboBridge, ros_gz_prefix, gz_launch_setup
from os.path import join, exists
from typing import Text, List, Iterable, Tuple, Union

def joint_state_publisher(launcher: BetterLaunch, use_gui=True, **node_args):
    '''
    Adds a joint_state_publisher / joint_state_publisher_gui with passed arguments as parameters
    Assumes some robot_description topic is published inside the namespace
    '''
    if isinstance(use_gui, bool):
        use_gui = str(use_gui)
    launcher.node('joint_state_publisher', parameters=node_args, condition='unless:' + use_gui)
    launcher.node('joint_state_publisher_gui', parameters=node_args, condition='if:' + use_gui)

def rviz(launcher: BetterLaunch, config_file=None, warnings=False):
    '''
    Runs RViz with the given config file and warning level
    '''
    args = [] if config_file is None else ['-d', config_file]
    if not warnings:
        args += ['--ros-args', '--log-level', 'FATAL']
    launcher.node('rviz2', 'rviz2', arguments=args)

def moveit(launcher: BetterLaunch, config_pkg: str, config_file: str):
    '''
    Launches a MoveIt configuration specified by package and launch file path
    '''
    moveit_launch_file_path = launcher.find(config_pkg, f'{config_file}.launch.py')
    launcher.include(config_pkg, moveit_launch_file_path)

def robot_description(launcher: BetterLaunch, package=None, description_file=None, description_dir=None, xacro_args=None):
    '''
    Returns the robot description after potential xacro parse if the file ends with xacro or xacro args are defined.
    '''
    description_file = launcher.find(package, description_file, description_dir)
    if description_file.endswith('urdf') and xacro_args is None:
        with open(description_file) as f:
            urdf_xml = f.read()
        return urdf_xml
    cmd = ['xacro', description_file] + (xacro_args if isinstance(xacro_args, list) else [xacro_args])
    try:
        with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as proc:
            stdout, stderr = proc.communicate()
            if proc.returncode == 0:
                return stdout
            else:
                print(f"Error processing xacro: {stderr}")
                return None
    except Exception as e:
        print(f"Failed to execute xacro command: {e}")
        return None

def robot_state_publisher(launcher: BetterLaunch, package: Optional[str] = None, description_file: Optional[str] = None, description_dir: Optional[str] = None, xacro_args: Optional[Dict[str, Any]] = None, **node_args: Dict[str, Any]):
    '''
    Add a robot state publisher node to the launch tree using the given description (URDF/Xacro) file.

    Args:
    - package: Name of the package containing the description file; if None, an absolute path is assumed.
    - description_file: Name of the URDF/Xacro file.
    - description_dir: Directory containing the file; if None, the location is derived.
    - xacro_args: Arguments passed to the Xacro processor.
    - node_args: Additional node arguments such as remappings or parameters.
    '''
    # Generate the robot description XML from URDF or Xacro files
    try:
        urdf_xml = robot_description(launcher, package, description_file, description_dir, xacro_args)
    except FileNotFoundError as e:
        print(f"Error loading robot description: {e}")
        return

    params = node_args.get('parameters', [])
    if not isinstance(params, list):
        params = [params]
    params.append({'robot_description': urdf_xml})
    node_args['parameters'] = params

    try:
        launcher.node("robot_state_publisher", **node_args)
    except Exception as e:
        print(f"Error launching robot state publisher node: {e}")

def declare_gazebo_axes(axes={}):
    '''
    Declares classical Gazebo axes as launch arguments.
    If axes is void then declares all 6 axes with default value 0.
    Otherwise declares the given axes with the given defaults.
    '''
    gz_axes = ('x', 'y', 'z', 'yaw', 'pitch', 'roll')
    if axes:
        filtered_axes = [axis for axis in gz_axes if axis in axes]
        declared_axes = {axis: axes[axis] for axis in filtered_axes}
    else:
        declared_axes = {axis: 0. for axis in gz_axes}

    return declared_axes

def gazebo_axes_args(declared_axes):
    '''
    Generate arguments corresponding to Gazebo spawner.
    '''
    axes_mapping = {'x': 'x', 'y': 'y', 'z': 'z', 'roll': 'R', 'pitch': 'P', 'yaw': 'Y'}
    args = []
    for axis, tag in axes_mapping.items():
        if axis in declared_axes:
            args.append(['-' + tag, declared_axes[axis]])
    flattened_args = [str(item) for sublist in args for item in sublist]
    return flattened_args

def create_gz_bridge(launcher: BetterLaunch, bridges: Union[GazeboBridge,List[Union[GazeboBridge,Tuple]]], name = 'gz_bridge'):
        '''
        Create a ros_gz_bridge::parameter_bridge with the passed GazeboBridge instances
        The bridge has a default name if not specified
        If any bridge is used for sensor_msgs/Image, ros_{gz,ign}_image will be used instead
        '''
        # adapt types
        if isinstance(bridges, GazeboBridge):
            bridges = [bridges]
        if len(bridges) == 0:
            return

        # lazy list of bridges
        for idx,bridge in enumerate(bridges):
            if not isinstance(bridge, GazeboBridge):
                bridges[idx] = GazeboBridge(*bridge)

        ros_gz = 'ros_' + ros_gz_prefix()

        # add camera_info for image bridges
        im_bridges = [bridge for bridge in bridges if bridge.is_image]

        for bridge in im_bridges:

            gz_head, gz_tail = bridge.gz_topic.split_tail()
            ros_head, ros_tail = bridge.ros_topic.split_tail()

            if not all(isinstance(tail, Text) and 'image' in tail for tail in (ros_tail, gz_tail)):
                continue

            cam = []
            for tail in (gz_tail, ros_tail):
                idx = tail.rfind('image')
                cam.append(tail[:idx] + 'camera_info')

            bridges.append(GazeboBridge(gz_head + [cam[0]], ros_head + [cam[1]], 'sensor_msgs/CameraInfo', GazeboBridge.gz2ros))

        std_config = sum([bridge.yaml() for bridge in bridges if not bridge.is_image], [])

        from tempfile import NamedTemporaryFile

        if std_config.has_elems():
            from tempfile import NamedTemporaryFile

            # Use YAML-based configuration, handles Gazebo topics that are invalid to ROS
            dst = NamedTemporaryFile().name

            # Construct the command using string formatting
            cmd = f'echo "{std_config}" >> {dst}'

            # Execute the command using subprocess.Popen
            process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            stdout, stderr = process.communicate()

            if process.returncode != 0:
                print(f"Error in command execution: {stderr}")
            else:
                launcher.node(f'{ros_gz}_bridge', 'parameter_bridge', name=name,
                            parameters={'config_file': dst})

        if len(im_bridges):
            # Use remapping to ROS topics
            remappings = []
            for bridge in im_bridges:
                for ext in ('', '/compressed', '/compressedDepth', '/theora'):
                    remapped_gz_topic = f'{bridge.gz_topic}{ext}'
                    remapped_ros_topic = f'{bridge.ros_topic}{ext}'
                    remappings.append((remapped_gz_topic, remapped_ros_topic))

            launcher.node(f'{ros_gz}_image', 'image_bridge', name=f'{name}_image',
                        arguments=[bridge.gz_topic for bridge in im_bridges],
                        remappings=remappings)

def gz_world_tf(launcher: BetterLaunch, world_frame = None):
        '''
        Runs a static_transform_publisher to connect `world` and Gazebo world name, if different from `world`
        '''
        if world_frame is None:
            world_frame = GazeboBridge.world()
        if world_frame != 'world':
            launcher.node('tf2_ros', 'static_transform_publisher',
                      name = 'gz_world_tf',
                      arguments = ['--frame-id', 'world','--child-frame-id', world_frame])

def create_gz_clock_bridge(GazeboBridge, name = 'gz_clock_bridge'):
        '''
        Create a ros_gz_bridge::parameter_bridge for the /clock topic
        Typically used in the launch file that runs the simulation before spawning things in
        '''
        return(GazeboBridge.clock(), name)


def gz_launch(launcher: BetterLaunch, world_file, gz_args = None, full_world = None, save_after = 5.):
        '''
        Wraps gz_sim_launch to be Ignition/GzSim agnostic
        default version is Fortress (6), will use GZ_VERSION if present
        if `full_world` is given as a raw `string` then the resulting SDF will be saved, including base world and spawned entities
        '''
        if isinstance(full_world, str):
            if exists(full_world):
                launch_file, launch_arguments = gz_launch_setup(full_world, gz_args)
            else:
                launch_file, launch_arguments = gz_launch_setup(world_file, gz_args)
                save_gz_world(launcher, full_world, save_after)
        else:
            launch_file, launch_arguments = gz_launch_setup(world_file, gz_args)
        return launcher.include(launch_file = launch_file, launch_arguments = launch_arguments)


# Currently working on.........................................................................................................

def save_gz_world(launcher: BetterLaunch, dst, after = 5.):
    '''
    Saves the current world under dst
    Resolves any spawned URDF through their description parameter and converts to SDF
    '''
    from .events import When
    with launcher.group(when = When(delay = after)):
        launcher.node('better_launch', 'generate_gz_world', arguments = [dst])

def spawn_gz_model(launcher: BetterLaunch, name, topic = 'robot_description', model_file = None, spawn_args = []):
        '''
        Spawns a model into Gazebo under the given name, from the given topic or file
        Additional spawn_args can be given to specify e.g. the initial pose
        '''

        if model_file is not None:
            spawn_args = flatten(spawn_args + ['-file',model_file,'-name', name])
        else:
            spawn_args = flatten(spawn_args + ['-topic',topic,'-name', name])

        pkg = 'ros_ign_gazebo' if ros_gz_prefix() == 'ign' else 'ros_gz_sim'
        launcher.node(package = pkg, executable = 'create', arguments=spawn_args)