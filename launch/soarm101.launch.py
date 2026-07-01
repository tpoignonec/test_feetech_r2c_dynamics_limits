"""Simple SOARM101 bringup launch file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch controller manager, state publisher, GUI controller and RViz."""
    robot_name = LaunchConfiguration('robot_name')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    usb_port = LaunchConfiguration('usb_port')

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare('test_feetech_r2c_dynamics_limits'),
            'config',
            'soarm101',
            'robot.xacro',
        ]
    )
    joint_config_file = PathJoinSubstitution(
        [
            FindPackageShare('test_feetech_r2c_dynamics_limits'),
            'config',
            'soarm101',
            'joint_config.yaml',
        ]
    )
    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare('test_feetech_r2c_dynamics_limits'),
            'config',
            'soarm101',
            'controllers.yaml',
        ]
    )

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                xacro_file,
                ' robot_name:=',
                robot_name,
                ' use_mock_hardware:=',
                use_mock_hardware,
                ' usb_port:=',
                usb_port,
                ' use_ros2_control:=true',
                ' joint_config_file:=',
                joint_config_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_file],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    joint_trajectory_gui = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution(
            [
                FindPackageShare('test_feetech_r2c_dynamics_limits'),
                'config',
                'soarm101',
                'config.rviz',
            ]
        )]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_name', default_value='soarm101'),
            DeclareLaunchArgument('use_mock_hardware', default_value='true'),
            DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            gripper_controller_spawner,
            joint_trajectory_gui,
            rviz,
        ]
    )
