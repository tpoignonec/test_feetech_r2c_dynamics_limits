"""Launch the standalone feetech single-motor node.

Example:
  ros2 launch test_feetech_r2c_dynamics_limits single_motor.launch.py \
      usb_port:=/dev/ttyACM0 joint_name:=gripper \
      open_position:=2046 close_position:=3491
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _int(name):
    """Return the launch argument ``name`` typed as an integer parameter."""
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _float(name):
    """Return the launch argument ``name`` typed as a float parameter."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _str(name):
    """Return the launch argument ``name`` typed as a string parameter."""
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    """Build the launch description for the single-motor node."""
    default_joint_config = PathJoinSubstitution([
        FindPackageShare('test_feetech_r2c_dynamics_limits'),
        'config', 'soarm101', 'joint_config.yaml',
    ])

    args = [
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'joint_config_file', default_value=default_joint_config),
        DeclareLaunchArgument('joint_name', default_value='gripper'),
        # open/close positions are in radians (0 rad = servo midpoint).
        DeclareLaunchArgument('open_position', default_value='0.8'),
        DeclareLaunchArgument('close_position', default_value='0.05'),
        # max_velocity is the goal speed in rad/s (applied on every
        # write_position).
        DeclareLaunchArgument('max_velocity', default_value='3.0'),
        # max_torque as a percentage of rated torque (0..100 %). Used for EPROM
        # config and applied dynamically on every write_position; -1 keeps the
        # YAML max_torque_limit value.
        DeclareLaunchArgument('max_torque', default_value='-1.0'),
        DeclareLaunchArgument('acceleration', default_value='50'),
        DeclareLaunchArgument('wait_seconds', default_value='5.0'),
        DeclareLaunchArgument('cycle_wait_seconds', default_value='5.0'),
    ]

    node = Node(
        package='test_feetech_r2c_dynamics_limits',
        executable='single_motor_node',
        name='feetech_single_motor',
        output='screen',
        parameters=[
            {
                'usb_port': _str('usb_port'),
                'joint_config_file': _str('joint_config_file'),
                'joint_name': _str('joint_name'),
                'open_position': _float('open_position'),
                'close_position': _float('close_position'),
                'max_velocity': _float('max_velocity'),
                'max_torque': _float('max_torque'),
                'acceleration': _int('acceleration'),
                'wait_seconds': _float('wait_seconds'),
                'cycle_wait_seconds': _float('cycle_wait_seconds'),
            }
        ],
    )

    return LaunchDescription([*args, node])
