# Sort-O — Autonomous Plastic Sorting Robot

**Robotics Studio 2 · 2026**

> Project website: https://jun-je0ng.github.io/Robotics-Studio-2

UR3e robotic arm + OnRobot RG2 gripper + Intel RealSense D435. Detects, classifies, and sorts plastic waste autonomously using YOLOv8 OBB detection and MoveIt2 motion planning.

---

## Dependencies

**System (Ubuntu 22.04)**
```bash
sudo apt install ros-humble-desktop ros-humble-ur ros-humble-moveit libnet1-dev
```

**Python**
```bash
pip install ultralytics pyrealsense2 opencv-python scipy numpy scikit-learn pillow
```

**Clone and build**
```bash
git clone https://github.com/Jun-Je0ng/Robotics-Studio-2.git
cd Robotics-Studio-2

vcs import src --input src/ur_onrobot/required.repos --recursive
rosdep install -y --from-paths src --ignore-src

colcon build --symlink-install
source install/setup.bash
```

---

## Camera Calibration

Run once after setup, and again whenever the camera or workspace board moves.

Place all 4 ArUco markers at the corners of the workspace board, then:

```bash
cd src/plastic_detection/plastic_detection/
python3 camera_calibration.py
```

This writes `camera_to_robot_calibration.json` to the same directory.

---

## Running — Real Robot

**Network check first:**
```bash
hostname -I          # get your laptop IP (192.168.0.x)
ping 192.168.0.197   # confirm robot is reachable
```

**Teach pendant setup (one-time):**
1. Installation → URCaps → External Control → set Host IP to your laptop IP
2. Program tab → URCaps → add External Control → press ▶ Play

**Terminal 1 — Robot driver**
```bash
source install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py \
    ur_type:=ur3e onrobot_type:=rg2 \
    robot_ip:=192.168.0.197 launch_rviz:=false
```
Wait for: `Robot connected to reverse interface. Ready to receive control commands.`
Then press ▶ Play on the teach pendant.

**Terminal 2 — MoveIt + motion controller**
```bash
source install/setup.bash
ros2 launch ur_gripper_demo ur_moveit.launch.py \
    ur_type:=ur3e demo_type:=reactive sim:=false
```

**Terminal 3 — Camera + detector**
```bash
source install/setup.bash
ros2 run plastic_detection obb_detector_node
```

**Terminal 4 — GUI**
```bash
source install/setup.bash
ros2 run ur_gripper_demo status_gui
```

> **Shortcut:** `ros2 launch ur_gripper_demo bringup.launch.py` launches all four with staggered delays.

---

## Running — Simulation (URSim)

**Start URSim:**
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```
Open Polyscope at `http://192.168.56.101:6080/vnc.html`

In Polyscope: Installation → URCaps → External Control → set Host IP to `192.168.56.1`
Then Program → add External Control → press ▶ Play

**Terminal 1 — Robot driver (simulator IP)**
```bash
source install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py \
    ur_type:=ur3e onrobot_type:=rg2 \
    robot_ip:=192.168.56.101 launch_rviz:=false
```

**Terminal 2 — MoveIt + motion controller (sim mode)**
```bash
source install/setup.bash
ros2 launch ur_gripper_demo ur_moveit.launch.py \
    ur_type:=ur3e demo_type:=reactive sim:=true
```

Terminals 3 and 4 are the same as the real robot above.
`sim:=true` skips gripper stall detection — the arm executes the full pick-place sequence without needing a physical object.

---

## Before pressing Start

Teach the robot where each bin is using Freedrive on the pendant, then save each pose via the GUI or terminal:

```bash
ros2 topic pub --once /motion_system/command std_msgs/msg/String "data: 'SAVE_BIN:pet_bottle'"
ros2 topic pub --once /motion_system/command std_msgs/msg/String "data: 'SAVE_BIN:hdpe_bottle'"
ros2 topic pub --once /motion_system/command std_msgs/msg/String "data: 'SAVE_BIN:pp_container'"
```

Poses are saved to `src/ur_gripper_demo/config/bin_poses.json` and loaded automatically on next launch.
