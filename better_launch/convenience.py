# TODO Create convenience module for common stuff like joint_state_publisher, robot_state_publisher, moveit, rviz, gazebo (see simple_launch)

from better_launch import BetterLaunch
import subprocess

class convenience_module:
    def __init__(self, launcher: BetterLaunch):
        self.launcher = launcher

    
    def joint_state_publisher(self, use_gui = True, **node_args):
        '''
        Adds a joint_state_publisher / joint_state_publisher_gui with passed arguments as parameters
        Assumes some robot_description topic is published inside the namespace
        '''
        if type(use_gui) == bool:
            use_gui = str(use_gui)

        self.launcher.node('joint_state_publisher', parameters = node_args, condition=UnlessCondition(use_gui))
        self.launcher.node('joint_state_publisher_gui', parameters = node_args, condition=IfCondition(use_gui))

    def rviz(self, config_file = None, warnings = False):
        '''
        Runs RViz with the given config file and warning level
        '''
        args = [] if config_file is None else ['-d', config_file]
        if not warnings:
            args += ['--ros-args', '--log-level', 'FATAL']
        self.launcher.node('rviz2', 'rviz2', arguments = args)


    def add_moveit(self, config_pkg: str, config_file: str):
        moveit_launch_file_path = self.launcher.find(config_pkg, f'{config_file}.launch.py')
        self.launcher.include(config_pkg, moveit_launch_file_path)

   
   import subprocess

def robot_description(self, package=None, description_file=None, description_dir=None, xacro_args=None):
    '''
    Returns the robot description after potential xacro parse if the file ends with xacro or xacro args are defined.
    '''
    # Retrieve the full path to the description file using a method that resolves the path based on the input parameters
    description_file = self.launcher.find(package, description_file, description_dir)

    # If the file is a URDF and no xacro arguments are provided, simply read and return the file content
    if type(description_file) == str and description_file.endswith('urdf') and xacro_args is None:
            with open(description_file) as f:
                urdf_xml = f.read()
            return urdf_xml

    cmd = ['xacro', description_file]
    if xacro_args:
        cmd += xacro_args if isinstance(xacro_args, list) else [xacro_args]
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

#currently working...............................
    def robot_state_publisher(self, package=None, description_file=None, description_dir=None,
                              xacro_args=None, **node_args):
        '''
        Add a robot state publisher node to the launch tree using the given description (urdf / xacro) file.

        * package -- is the name of the package that contains the description file (if None then assume an absolute description file)
        * description_file -- is the name of the urdf/xacro file
        * description_dir -- the name of the directory containing the file (None to have it found)
        * xacro_args -- arguments passed to xacro (will force use of xacro)
        * node_args -- any additional node arguments such as remappings or parameters
        '''

        urdf_xml = robot_description(package, description_file, description_dir, xacro_args)

        if 'parameters' in node_args:
            # already some parameters, change to list of dictionaries
            node_args['parameters'] = adapt_type(node_args['parameters'], NODE_PARAMS)
            + [{'robot_description': urdf_xml}]
        else:
            node_args['parameters'] = [{'robot_description': urdf_xml}]

        # Launch the robot state publisher with the desired URDF
        self.launcher.node("robot_state_publisher", **node_args)


    def add_gazebo(self, world_file: str, paused: bool = True):
        gazebo_args = {
            'world': world_file,
            'paused': 'true' if paused else 'false'
        }
        self.launcher.node(pkg='gazebo_ros', exec='gzserver', name='gazebo', node_args=gazebo_args)
        self.launcher.node(pkg='gazebo_ros', exec='gzclient', name='gazebo_gui')

