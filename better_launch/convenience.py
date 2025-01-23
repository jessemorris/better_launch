# TODO Create convenience module for common stuff like joint_state_publisher, robot_state_publisher, moveit, rviz, gazebo (see simple_launch)

from better_launch import BetterLaunch
import subprocess

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

def robot_state_publisher(launcher: BetterLaunch, package=None, description_file=None, description_dir=None, xacro_args=None, **node_args):
    '''
        Add a robot state publisher node to the launch tree using the given description (urdf / xacro) file.

        * package -- is the name of the package that contains the description file (if None then assume an absolute description file)
        * description_file -- is the name of the urdf/xacro file
        * description_dir -- the name of the directory containing the file (None to have it found)
        * xacro_args -- arguments passed to xacro (will force use of xacro)
        * node_args -- any additional node arguments such as remappings or parameters
        '''
    urdf_xml = robot_description(launcher, package, description_file, description_dir, xacro_args)
    if 'parameters' in node_args:
        if isinstance(node_args['parameters'], list):
            node_args['parameters'].append({'robot_description': urdf_xml})
        else:
            node_args['parameters'] = [node_args['parameters'], {'robot_description': urdf_xml}]
    else:
        node_args['parameters'] = [{'robot_description': urdf_xml}]
    launcher.node("robot_state_publisher", **node_args)

