import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('eyou_robot_control')

    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Get URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'eyou_robot.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {"robot_description": robot_description_content}

    # Get controller config file path
    controllers_file = os.path.join(pkg_share, 'config', 'eyou_controllers.yaml')

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Left arm controller spawner
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Right arm controller spawner
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Left gripper controller spawner
    #left_gripper_controller_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["left_gripper_controller", "--controller-manager", "/controller_manager"],
    #)

    # Right gripper controller spawner
    #right_gripper_controller_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["right_gripper_controller", "--controller-manager", "/controller_manager"],
    #)

    # Delay start of spawners until controller_manager is running
    delay_spawners_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                left_arm_controller_spawner,
                right_arm_controller_spawner,
                left_gripper_controller_spawner,
                right_gripper_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        declared_arguments +
        [
            robot_state_publisher_node,
            controller_manager_node,
            delay_spawners_after_controller_manager,
        ]
    )
