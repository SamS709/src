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

    camera_controller_node = Node(
        package="my_cv_package",
        executable="face_detecter_pi",
    )

    

    return LaunchDescription([
        #joint_state_publisher_gui_node,
        camera_controller_node
    ])
