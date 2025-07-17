from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import PathJoinSubstitution, FindExecutable, FindPackageShare
import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

# ARGUMENTS

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop",
            default_value="true",
            description="Start teleop_twist_keyboard for manual control.",
        )
    )

    teleop = LaunchConfiguration("teleop")

# CONFIGS

    urdf_path = os.path.join(get_package_share_path('rasptank_description'),'urdf','rasptank.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('rasptank_description'), 'rviz', 'my_robot_config.rviz')

    robot_description = ParameterValue(Command(['xacro ',urdf_path]),value_type=str) # la commande xacro va transformer lefichier .xacro en fichier .urdf

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    other_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch/gazebo.launch.py" ) 
        )
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen")
    

    # displays camera output seen by the robot
    get_camera = Node(
        package = "my_cv_package",
        executable= "face_detecter"
    )

    # allows the user to teleop the robot
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",
        remappings=[("/cmd_vel", "/cmd_vel")],
        parameters=[{"stamped": True}],
        condition=IfCondition(teleop),
    )
    
   
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )



    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
    )

    return LaunchDescription([
        #joint_state_publisher_gui_node,
        #other_launch_file,
        #spawn_robot,
        robot_state_publisher_node,
        rviz2_node,
        get_camera
    ])
