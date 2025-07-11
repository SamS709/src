from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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
    
    get_camera = Node(
        package = "my_cv_package",
        executable= "face_detecter"
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
