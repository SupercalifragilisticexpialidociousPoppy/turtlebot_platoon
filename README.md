# turtlebot_platoon

ROS 2 (`rclpy`) controller package for autonomous LiDAR-based TurtleBot3 platooning.

## Package Layout

- `platoon_controller/scan_filter.py` *(Role 2)*: Filters `/scan` (`LaserScan`) into a tracked target point (`platoon/target`) using nearest-face clustering, outlier/jump rejection, EMA smoothing, and a dynamic bearing-centered tracking window for corners.
- `platoon_controller/distance_controller.py` *(Role 3)*: Linear 1D PD gap controller (`cmd_vel` `linear.x` only) for the Milestone 2 dummy-wall distance-lock simulation.
- `platoon_controller/platoon_follower.py` *(Roles 3 + 4)*: Unified follower node combining linear PD distance control with Pure Pursuit angular steering (`kappa = 2*y / (x^2 + y^2)`) and stationary bearing alignment.
- `platoon_controller/control_math.py`: Pure-Python control and LiDAR clustering helpers.
- `platoon_controller/wall_driver.py` & `models/dummy_wall.sdf`: Gazebo dummy wall testbed for Milestone 2.
- `launch/platoon_follower.launch.py`: Launches `scan_filter` + `platoon_follower` (with optional `namespace`).

## Build & Test (Ubuntu / ROS 2 Humble)

```bash
colcon build --packages-select platoon_controller
source install/setup.bash
colcon test --packages-select platoon_controller
```

To run the pure-Python unit tests without ROS 2 (e.g., on macOS):

```bash
PYTHONPATH=platoon_controller python3 -m unittest discover -s platoon_controller/test -p "test_platoon_*.py" -v
```

## Running

### Milestone 2: Single-Robot Distance-Lock (Dummy Wall)
```bash
ros2 run platoon_controller scan_filter
ros2 run platoon_controller distance_controller
ros2 run platoon_controller wall_driver
```

### Milestone 3 / 4: Coupled Follower (Linear PD + Pure Pursuit)
```bash
# Default namespace (/scan, /cmd_vel)
ros2 launch platoon_controller platoon_follower.launch.py use_sim_time:=true

# Namespaced follower robot (e.g., /tb3_2/scan, /tb3_2/cmd_vel)
ros2 launch platoon_controller platoon_follower.launch.py namespace:=tb3_2 use_sim_time:=true
```
