# turtlebot_platoon

LiDAR-only autonomous leader-following platoon for **TurtleBot3 Burger** robots running **ROS 2 Humble** (`rclpy`). Every follower operates as a reactive agent using only its own 2D LiDAR (`/scan` → `/cmd_vel`) — no inter-robot wireless communication, cameras, or global maps.

---

## Architecture & Package Layout

Data flow on every follower robot:
```text
scan (LaserScan) ─▶ scan_filter ─▶ platoon/target (PointStamped) ─▶ platoon_follower ─▶ cmd_vel (Twist)
```
All topic names are relative, allowing single-robot root namespaces or multi-robot namespacing (`namespace:=tb3_2`).

- **`platoon_controller/scan_filter.py`** *(Perception / Role 2)*:
  Wraps LDS-01 angles from `[0, 2π]` to `(-π, π]`, keeps valid beams in `[0.12, 1.2]` m inside a `±15°` sector, centers the window on the last locked bearing (clamped to `±40°`, and locked forward at `0°` below `0.20 m`), anchors clustering on the `k`-th nearest beam (`k = 3`, tolerance `0.10 m`) to reject single-beam outliers, smooths `(distance, bearing)` via EMA (`α = 0.4`), rejects sudden jumps `> 0.75 m`, bridges up to `3` missed frames, and resets state across `> 1.0 s` `/scan` stalls.
- **`platoon_controller/platoon_follower.py`** *(Gap + Steering / Roles 3 & 4)*:
  - **3-zone gap control (`0.30 m` target, `0.20 m` reverse threshold):**
    - `0.30 m` to `1.20 m`: PD forward tracking (`kp = 2.0`, `kd = 0.5`, derivative low-pass `α = 0.3`, clamped `>= 0` so derivative noise never backs away from far targets).
    - `0.20 m` to `0.30 m`: hold zone (`linear.x = 0.0`) while gently aligning angular bearing.
    - `< 0.20 m`: straight reverse backoff (`no_turn_distance = 0.20 m`, immediate `-0.08 m/s` emergency backoff below `0.16 m`).
  - **Pure Pursuit steering:** Arc curvature `κ = 2y / (x² + y²)`, `ω = vκ`, blended smoothly into proportional alignment (`k_align = 1.4`) below `0.08 m/s`, with curvature speed reduction (`turn_slowdown_gain = 0.5`).
  - **Actuation & safety:** Differential-drive per-wheel speed scaling (`max_wheel_speed = 0.22 m/s`, `wheel_separation = 0.160 m`), linear/angular acceleration rate limits (`0.6 m/s²`, `2.0 rad/s²` to prevent 5 Hz LDS scan smearing), `0.6 s` target-loss stop (resetting derivative state), live `ros2 param set` tuning, `invert_linear` / `invert_angular` hardware flags, and optional `log_csv` recording.
- **`platoon_controller/control_math.py`**: Pure-Python perception and control math (no `rclpy` dependency for cross-platform unit testing).
- **`platoon_controller/distance_controller.py`**, **`wall_driver.py`**, & **`models/dummy_wall.sdf`**: Milestone 2 1D PD controller and Gazebo Classic moving-wall testbed.
- **`launch/platoon_follower.launch.py`**: Launches `scan_filter` + `platoon_follower` with arguments `namespace`, `use_sim_time`, `target_distance`, `max_track_range`, `invert_linear`, `invert_angular`, and `log_csv`.
- **`docs/project_report.tex`** & **`docs/Robot_Platoon_AI_Project_Proposal.docx`**: Final CS F407 report and original project proposal.

---

## Physical Robot Build (Hardware Setup)

Our fleet consists of **4 TurtleBot3 Burger** platforms (Raspberry Pi SBC, OpenCR controller board, DYNAMIXEL XL430 wheel actuators, and 360° LDS-01 LiDAR). Hardware assembly and OS/firmware setup followed the official ROBOTIS documentation:
- **ROBOTIS TurtleBot3 e-Manual (Overview & Assembly):** https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
- **TurtleBot3 Quick Start / SBC & OpenCR Setup (ROS 2 Humble):** https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
- **ROS 2 Humble Documentation:** https://docs.ros.org/en/humble/

**Field modification for platooning:** Because all 4 TurtleBot3s have their LDS-01 turrets mounted at the same physical height, lightweight makeshift height extensions were added onto the chassis so each follower's LiDAR reliably catches a wide, consistent face on the robot directly ahead (and avoids dropping below `min_cluster_points = 3` at `0.8–1.2 m`).

---

## Build & Unit Tests

### On Ubuntu / ROS 2 Humble (Robot Pi or Dev Machine)
```bash
cd ~/platoon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select platoon_controller
source install/setup.bash
colcon test --packages-select platoon_controller
colcon test-result --verbose
```
*(You can also run `./scripts/test_platoon_ros2.sh` from the workspace root; see `TEST_VERIFICATION.md` for the full Gazebo checklist.)*

### Pure-Python Unit Tests Without ROS 2 (e.g., macOS)
```bash
PYTHONPATH=platoon_controller python3 -m unittest discover -s platoon_controller/test -p "test_platoon_*.py" -v
```

---

## Physical Robot Bring-Up & Platoon Execution (3-Step Procedure)

### Step 1: Establish SSH Connection (Laptop ↔ Robot)
1. Power on each TurtleBot3 Burger (ensure charged 3S LiPo batteries; low battery voltage sags the USB 5V rail and stalls the LDS-01 mid-run) and connect both the robot Raspberry Pi and your laptop to the **same Wi-Fi network**.
2. Assign each robot a **unique `ROS_DOMAIN_ID`** (e.g., `30`, `31`, `32`, `33`) in `~/.bashrc` so robots on the same Wi-Fi do not hijack each other's `/scan` and `/cmd_vel` topics.
3. SSH into each robot from your laptop (and optionally sync code via `scp` or `git pull`):
   ```bash
   ssh teamfound@<ROBOT_IP>

   # Example: copy updated package from laptop to robot workspace
   scp -r platoon_controller teamfound@<ROBOT_IP>:~/platoon_ws/src/
   ```

### Step 2: Source Environment & Bring Up TurtleBot3 Hardware (Window 1 per Robot)
Before launching, confirm `~/.bashrc` sets the robot's environment variables (`ROS_DISTRO=humble`, `TURTLEBOT3_MODEL=burger`, `LDS_MODEL=LDS-01`, and that robot's `ROS_DOMAIN_ID`), and clear any orphaned serial/DDS processes from previous sessions:
```bash
pkill -9 -f "hlds_laser_publisher|turtlebot3_ros|robot_state_publisher|scan_filter|platoon_follower" 2>/dev/null || true
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

source ~/.bashrc
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01
# Optional when running standalone on a single robot to isolate DDS from Wi-Fi:
# export ROS_LOCALHOST_ONLY=1

ros2 launch turtlebot3_bringup robot.launch.py
```
Wait until `hlds_laser_publisher` (`/dev/ttyUSB0`) and `turtlebot3_ros` / `diff_drive_controller` (`/dev/ttyACM0`) complete gyro calibration and print `Run!`. Keep this terminal running so `/dev/ttyUSB0` stays open cleanly between follower restarts.

*(Optional verification in a second SSH tab: `ros2 topic echo /scan --qos-reliability best_effort --once | head -n 15`.)*

### Step 3: Run Leader Teleop (Robot 1) & Platoon Followers (Robots 2–4)

Open a second SSH terminal on each robot, source ROS 2 and the workspace (`cd ~/platoon_ws && source /opt/ros/humble/setup.bash && source install/setup.bash`), then:

- **On the Leader (Robot 1):** drive manually with keyboard teleoperation:
  ```bash
  ros2 run turtlebot3_teleop teleop_keyboard
  ```
  *(Controls: `w`/`x` forward/back, `a`/`d` turn left/right, `s` stop.)*

- **On Followers (Robots 2, 3, and 4):** launch the coupled `scan_filter` + `platoon_follower` stack:
  ```bash
  ros2 launch platoon_controller platoon_follower.launch.py
  ```
  - **Motor direction check:** Always verify `w`/`x`/`a`/`d` in `teleop_keyboard` once per chassis. If a robot drives backward on `+linear.x` (`w` goes backward), pass `invert_linear:=true` (or `invert_angular:=true` if turning is reversed):
    ```bash
    ros2 launch platoon_controller platoon_follower.launch.py invert_linear:=true
    ```
  - **CSV calibration logging:**
    ```bash
    ros2 launch platoon_controller platoon_follower.launch.py log_csv:=/tmp/platoon_run.csv
    ```
  - **Live gain tuning while running:**
    ```bash
    ros2 param set /platoon_follower kp 2.0
    ros2 param set /platoon_follower kd 0.5
    ros2 param set /platoon_follower k_align 1.4
    ```

---

## Simulation Testing (Gazebo 11)

### Milestone 2: Single-Robot Distance-Lock (Dummy Wall)
```bash
ros2 run platoon_controller scan_filter
ros2 run platoon_controller distance_controller
ros2 run platoon_controller wall_driver
```

### Milestone 3 / 4: Namespaced Follower in Gazebo
```bash
ros2 launch platoon_controller platoon_follower.launch.py use_sim_time:=true
# Namespaced second robot (/tb3_2/scan, /tb3_2/cmd_vel):
ros2 launch platoon_controller platoon_follower.launch.py namespace:=tb3_2 use_sim_time:=true
```
