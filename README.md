# test_feetech_r2c_dynamics_limits

## Installation

```bash
mkdir -p ws_ros/src/external
cd ws_ros
vcs import src/external < src/test_feetech_r2c_dynamics_limits/test_feetech_r2c_dynamics_limits.repos

rosdep install --from-paths src -i -r -y

colcon build --symlink-install
```

## Launch

```bash
source install/setup.bash

ros2 launch test_feetech_r2c_dynamics_limits soarm101.launch.py  \
    use_mock_hardware:=true
```

To open close the gripper:

```bash
export GRIPPER_POSITION_OPEN=0.8
export GRIPPER_POSITION_CLOSED=0.05

# Open
ros2 run test_feetech_r2c_dynamics_limits gripper_command --ros-args -p position:=$GRIPPER_POSITION_OPEN

# Close
ros2 run test_feetech_r2c_dynamics_limits gripper_command --ros-args -p position:=$GRIPPER_POSITION_CLOSED

# By default, max_vel and max_effort are set to -1, which sends empty vectors
# to the ParallelGripperCommand action.

# Test limits
ros2 run test_feetech_r2c_dynamics_limits gripper_command --ros-args \
    -p position:=$GRIPPER_POSITION_CLOSED \
    -p max_vel:=0.5 \
    -p max_effort:=1.0

# Add verbose gripper logs
ros2 run test_feetech_r2c_dynamics_limits gripper_command --ros-args \
    -p position:=0.5 \
    -p max_vel:=0.5 \
    -p max_effort:=1.0
```