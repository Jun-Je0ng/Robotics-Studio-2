# Robotics-Studio2

## Simulation (MoveIt + URSim)
1. Start URSim:
`ros2 run ur_client_library start_ursim.sh -m ur3e`
2. Open Polyscope in browser:
   http://192.168.56.101:6080/vnc.html
   Start Robot
   Program → URCaps → External Control
   Press ▶ Play (or "Play from selection") on the teach pendant
3. Start the driver (connect to simulator IP):
`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true`
   The IP above is the simulator IP.
4. Start MoveIt:
`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true`

---

## Real Robot (UR3e + MoveIt)
1. Turn on the robot power and release brakes.
2. Get your laptop IP:
`hostname -I`
   Use the IP starting with `192.168.0.xxx` (example: 192.168.0.193).
3. On the teach pendant:
   Installation → URCaps → External Control
   Enter your laptop IP for both Hostname and IP (Hostname and IP is samething).
   Make sure **Simulation mode is OFF**.
4. Press ▶ Play (or "Play from selection") on the teach pendant.
5. Start the driver (Controller) (robot IP example: 192.168.0.197):
`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.197 launch_rviz:=true`
6. Start MoveIt:
`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true`

---

## Network Check
Get laptop IP:
`hostname -I`
Ping robot:
`ping 192.168.0.197`
Ping only checks basic network connectivity — it does NOT mean MoveIt is connected.

---

## ML / Perception — Plastic Detection (ROS2 Nodes)

### Overview
The plastic detection system uses a YOLOv8 OBB (Oriented Bounding Box) model
to detect and classify plastic bottles (PET and HDPE) in real time using an
Intel RealSense RGB-D camera. It publishes detections and grip poses as JSON
over ROS2 topics.

### Prerequisites

**Hardware:**
- Intel RealSense RGB-D camera connected via USB

**Python dependencies:**
```bash
pip install ultralytics pyrealsense2 opencv-python scipy numpy
```

**Model weights** — not included in the repo.
Download `best.pt` from the shared Google Drive folder:
> [Download best.pt — Google Drive](https://drive.google.com/file/d/1E8UziEcsdmICbjATS0gKJ9bxvSQeJJfW/view)

Once downloaded, place it anywhere on your machine, then open
`src/plastic_detection/plastic_detection/obb_detector_node.py`
and update this line at the top to match where you saved it:
```python
MODEL_PATH = "/your/path/to/best.pt"
```

**Camera-to-robot calibration** — must be generated per setup.
For initial testing you can use Rian's calibration file — note this is only
valid if the camera and robot are in the exact same position as Rian's setup:
> [Download camera_to_robot_calibration.json — Google Drive](https://drive.google.com/file/d/1WYGAGIuWkN1SfdV2ZPyGIjgIhSolGRTG/view?usp=sharing)

Once downloaded, place it anywhere on your machine, then open
`src/plastic_detection/plastic_detection/obb_detector_node.py`
and update this line at the top to match where you saved it:
```python
CALIBRATION_FILE = "/your/path/to/camera_to_robot_calibration.json"
```

> **Important:** For real deployment you must generate your own calibration
> file using `camera_calibration.py` — see the Calibration section below.

### Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select plastic_detection
source install/setup.bash
```

### Running the nodes

**Terminal 1 — OBB Detector Node**
Captures RealSense frames, runs OBB detection, publishes detections to `/plastic_detections`:
```bash
ros2 run plastic_detection obb_detector_node
```

**Terminal 2 — Grip Pose Node**
Subscribes to `/plastic_detections`, computes grip pose for the parallel jaw gripper, publishes to `/grip_pose`:
```bash
ros2 run plastic_detection grip_pose_node
```

**Terminal 3 — Monitor detections (optional)**
```bash
ros2 topic echo /plastic_detections
```

**Terminal 4 — Monitor grip poses (optional)**
```bash
ros2 topic echo /grip_pose
```

### ROS2 Topics

| Topic | Direction | Description |
|---|---|---|
| `/plastic_detections` | Published | Stable detected objects with pose, dimensions, classification |
| `/grip_pose` | Published | Computed grip pose for each detected object |

### /plastic_detections message format
Each message is a JSON array. Each object contains:
```json
[{
  "pose": {
    "position": {"x": 22.1, "y": -306.7, "z": -133.1},
    "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.71, "qw": 0.71}
  },
  "dimensions": {"dx_mm": 65.2, "dy_mm": 210.5, "dz_mm": 63.0},
  "classification": {"class": "pet_bottle", "confidence": 0.94},
  "debug": {
    "z_table_mm": -183.1,
    "z_approach_mm": -33.1,
    "angle_deg": 45.0,
    "angle_rad": 0.7854,
    "depth_m": 0.312,
    "dz_source": "depth"
  }
}]
```

> **Note:** `dz_mm` may be absent from `dimensions` if the depth-based
> diameter measurement failed for that frame.

### /grip_pose message format
```json
[{
  "class": "pet_bottle",
  "confidence": 0.94,
  "grip_position": {"x": 22.1, "y": -306.7, "z": -133.1},
  "grip_orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.71, "qw": 0.71},
  "jaw_opening_mm": 75.0,
  "approach": "side_down",
  "debug": {
    "angle_deg": 45.0,
    "is_upright": false,
    "long_axis_mm": 210.5,
    "short_axis_mm": 65.2,
    "jaw_dir": [0.7071, -0.7071, 0.0],
    "approach_vec": [0.0, 0.0, -1.0]
  }
}]
```

### Configuration
Key parameters can be adjusted at the top of each node file:

**obb_detector_node.py**

| Parameter | Default | Description |
|---|---|---|
| `MODEL_PATH` | see file | Path to downloaded `best.pt` — update to your path |
| `CALIBRATION_FILE` | see file | Path to calibration JSON — update to your path |
| `CONF_THRESHOLD` | 0.7 | Minimum confidence to accept a detection |
| `STABILITY_FRAMES` | 5 | Frames object must be detected before publishing |
| `GRIP_OFFSET` | 50mm | Height above table surface to grip |
| `APPROACH_OFFSET` | 150mm | Height above table surface to approach |
| `SHOW_DISPLAY` | True | Show live camera feed — comment out to disable |

**grip_pose_node.py**

| Parameter | Default | Description |
|---|---|---|
| `UPRIGHT_THRESHOLD_DEG` | 20° | Angle below which bottle is considered upright |
| `JAW_SAFETY_MARGIN_MM` | 10mm | Extra margin added to jaw opening width |

### Calibration
Camera-to-robot calibration must be run once per setup whenever the camera
or robot position changes. It generates a `camera_to_robot_calibration.json`
file that maps camera coordinates to robot base frame coordinates.

**Steps:**
1. Print ArUco markers (4x4_50 dictionary, IDs 0-3) at 60x60mm each
2. Stick them at the 4 corners of the workspace plywood inside the guard rails
3. Run the calibration script:
```bash
python3 camera_calibration.py
```
4. Follow the on-screen instructions — move the robot to each marker and
   enter the teach pendant XYZ coordinates when prompted
5. Press `C` to compute and save the calibration once all points are captured
6. Update `CALIBRATION_FILE` in `obb_detector_node.py` to point to the
   generated JSON file

> **Tip:** Use at least 4 calibration points spread across the workspace
> for best accuracy. More points = better calibration.