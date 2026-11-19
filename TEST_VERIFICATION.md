# Test & Gazebo Verification Guide (Linux / ROS 2)

> **Status:** Test verification checklist for **PR #2 (`feat/pure-pursuit-follower`)**.  
> Run these commands on an Ubuntu + ROS 2 Humble machine with Gazebo before merging or deploying to physical TurtleBot3s.

---

## 1. Build & Run Automated Tests

Run from the root of your ROS 2 workspace (or run `./scripts/test_platoon_ros2.sh` from the repository root):

```bash
colcon build --packages-select platoon_controller
source install/setup.bash
colcon test --packages-select platoon_controller
colcon test-result --verbose
```

Expected checks in `colcon test`:
- `test_flake8` & `test_pep257` lint checks pass
- `test_platoon_control.py` (14 pure-Python Pure Pursuit, PD, and LiDAR clustering tests) passes

---

## 2. Gazebo Runtime Verification

### A. Single-Robot Follower Launch (Default Namespace: `/scan`, `/cmd_vel`)
```bash
ros2 launch platoon_controller platoon_follower.launch.py use_sim_time:=true
```

### B. Multi-Robot / Twin-Robot Launch (Namespaced Follower: `/tb3_2/scan`, `/tb3_2/cmd_vel`)
```bash
ros2 launch platoon_controller platoon_follower.launch.py namespace:=tb3_2 use_sim_time:=true
```

### C. Optional CSV Logging for Calibration Report
```bash
ros2 launch platoon_controller platoon_follower.launch.py \
  use_sim_time:=true \
  log_csv:=/tmp/platoon_run.csv
```

Logs columns: `t,distance,bearing_deg,error,d_filtered,linear_x,angular_z`.

---

## 3. What to Verify in Gazebo
- [ ] **Straightaway distance lock:** `platoon_follower` holds ~`0.30 m` gap against a moving target (`dummy_wall` or lead TurtleBot3) with minimal overshoot.
- [ ] **Cornering (Pure Pursuit):** When the lead TurtleBot3 turns left/right beyond `±15°`, `scan_filter` keeps lock via the dynamic bearing-centered window and `platoon_follower` steers smoothly along the curve.
- [ ] **Safety gates:** Stopping the target or removing it from view triggers `No target -- holding still.` and publishes zero velocity; moving closer than `0.16 m` triggers straight emergency backoff.
