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

# DECLARE ARGUMENTS

    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop",
            default_value="false",
            description="Start teleop_twist_keyboard for manual control.",
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    teleop = LaunchConfiguration("teleop")

# CAMERA

    # Publishes camera data
    camera_publisher_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera_node",
        parameters=[{
            "image_size": [640, 480],}])

    # Subscribes to camera data to look at humans
    camera_controller_node = Node(
        package="my_cv_package",
        executable="face_detecter_pi",
    )

# ULTRASONIC SENSOR

    # Publishes ultrasonic sensor data
    ultrasonic_sensor_publisher_node = Node(
        package="rasptank_ultrasonic_sensor",
        executable="ultrasonic_sensor_publisher",
        name="ultrasonic_sensor_publisher",
    )

    # Subscribes to ultrasonic sensor data to stop if an obstacle is too close

    
# DIFF DRIVE CONTROLLER

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rasptank_diff_drive"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rasptank_diff_drive"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    motor_controller_node = Node(
        package="rasptank_diff_drive",
        executable="motor_controller.py",
        name="motor_controller",
        output="both",
    )
    

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        camera_publisher_node,
        camera_controller_node,
        ultrasonic_sensor_publisher_node,
        control_node,
        robot_state_pub_node,
        motor_controller_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

