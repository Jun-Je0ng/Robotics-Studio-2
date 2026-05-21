# Status GUI — UR3e + RG2 Control Panel

A tkinter-based operator interface for the UR3e arm and RG2 gripper pick-and-place system.
It displays live system state, drives the motion controller via ROS2 topics, and shows the
annotated camera feed from the OBB detector.

## Running

```bash
# Source the workspace first
source install/setup.bash

ros2 run ur_gripper_demo status_gui
```

The full demo requires three terminals:

```bash
# Terminal 1 — MoveIt + motion controller
ros2 launch ur_gripper_demo ur_moveit.launch.py

# Terminal 2 — OBB detector (camera + YOLO)
ros2 run plastic_detection obb_detector_node

# Terminal 3 — GUI
ros2 run ur_gripper_demo status_gui
```

## Window Layout

```
┌─────────────────────────────────────────────────────────────────┐
│  UR3e + RG2  Control Panel                          ●  ONLINE   │
├──────────────┬──────────────────────────┬───────────────────────┤
│  LEFT PANEL  │      MIDDLE PANEL        │     RIGHT PANEL       │
│              │                          │                       │
│  SEQUENCE    │  SYSTEM  ROBOT  OBJ  SEQ │  CAMERA FEED          │
│  [START][STP]│  ──────────────────────  │  (annotated frame)    │
│  [  Home   ] │                          │                       │
│              │  PICK-PLACE CYCLE        │  PERCEPTION           │
│  GRIPPER     │  ●─○─○─○─○─○─○─○        │  detections + list    │
│  [Open][Cls] │                          │                       │
│              │  EVENT LOG               │  PICK PRIORITY        │
│  POSES       │  [timestamped events...] │  ○ Any                │
│  [Save Pose] │                          │  ○ pet_bottle   0.93  │
│  [Save File] │                          │  ○ hdpe_bottle  —     │
│  [Load File] │                          │                       │
│  [Clear All] │                          │  BIN POSES            │
│              │                          │  [pet_bottle]         │
│              │                          │  [hdpe_bottle]        │
│              │                          │  [custom entry][Save] │
│              │                          │  [↺ Reload Bins]      │
│              │                          │                       │
│              │                          │  MOTION SYSTEM        │
├──────────────┴──────────────────────────┴───────────────────────┤
│  ⚠  E-STOP          ↺  Reset        Emergency stop halts...     │
└─────────────────────────────────────────────────────────────────┘
```

## Controls Reference

### SEQUENCE (left panel)

| Button | Command sent | Effect |
|--------|-------------|--------|
| **▶ START** | `start` | Begins the reactive pick-and-place cycle |
| **■ STOP** | `stop` | Stops after the current pick completes; does not interrupt mid-grasp |
| **⌂ Home** | `home` | Returns the arm to the `up` named pose |

### GRIPPER (left panel)

| Button | Command sent | Effect |
|--------|-------------|--------|
| **◁▷ Open** | `open_gripper` | Opens the RG2 to ~110 mm |
| **▷◁ Close** | `close_gripper` | Closes the RG2 to near-zero |

> These pass through `handleCommand` in the motion controller. Wire up
> `open_gripper` / `close_gripper` there if they are not yet implemented.

### POSES (left panel)

Forwards `save_pose`, `save_poses_file`, `load_poses`, and `clear_poses` commands
to the motion controller's `handleCommand`. These are placeholders — implement the
handlers in `motion_controller_reactive.cpp` if needed.

### PICK PRIORITY (right panel)

Radio buttons set which bottle type the arm prefers to pick first.

| Selection | Command sent |
|-----------|-------------|
| Any (closest first) | `TARGET_CLASS:` (empty) |
| pet\_bottle | `TARGET_CLASS:pet_bottle` |
| hdpe\_bottle | `TARGET_CLASS:hdpe_bottle` |

Live confidence scores (from `/plastic_detections`) are shown next to each type.
The confidence display updates every detection frame; `—` means the type is not
currently detected.

### BIN POSES (right panel)

Used to teach the drop-off position for each bottle type.

1. Jog the arm so the gripper is centred over the correct bin.
2. Click the type button (e.g. **pet\_bottle**) to save the current end-effector
   pose to `config/bin_poses.json`.
3. **↺ Reload Bins** re-reads the JSON without restarting the controller.

| Button | Command sent |
|--------|-------------|
| **pet\_bottle** | `SAVE_BIN:pet_bottle` |
| **hdpe\_bottle** | `SAVE_BIN:hdpe_bottle` |
| **Save** (custom) | `SAVE_BIN:<typed name>` |
| **↺ Reload Bins** | `LOAD_BINS` |

### E-STOP bar (bottom)

| Button | Command sent | Effect |
|--------|-------------|--------|
| **⚠ E-STOP** | `estop` | Calls `arm.stop()` immediately; sets stop flag |
| **↺ Reset** | `reset` | Calls `arm.stop()`, clears stop flag, requests home |

## Pick-Place Cycle Diagram (middle panel)

A horizontal stepper shows which stage of the pick cycle is active.

```
● IDLE  ○ HOME  ○ WAIT  ○ PICK  ○ GRASP  ○ RAISE  ○ PLACE  ○ DONE
```

| Node colour | Meaning |
|-------------|---------|
| Grey (filled) | IDLE — waiting for START |
| Blue | HOMING — arm returning to home |
| Cyan | WAITING — waiting for perception frame |
| Yellow | PREGRASP — planning and moving above object |
| Orange | GRASPING — descending and closing gripper |
| Cyan | RAISING — lifting object |
| Cyan | TRANSPORTING — moving to bin and releasing |
| Green | DONE — cycle complete |
| **Red** | FAILED — gripper found nothing; will retry |

Past nodes dim; the current node is filled bright. Resizes automatically
with the window.

## Status Cards (middle panel)

Four compact cards update in real time from ROS topics:

| Card | Topic | What it shows |
|------|-------|---------------|
| SYSTEM | `/system_status` | Running / Stopped / Failed |
| ROBOT | `/robot_state` | Current motion state string |
| OBJECT | `/motion_system/current_object` | ID of the object being processed |
| SEQUENCE | `/motion_system/status` | Last status message from the controller |

## Event Log (middle panel)

Colour-coded timestamped log of every received event and every command the GUI sends.
Click **Clear** to wipe it. Colours:

| Colour | Meaning |
|--------|---------|
| Green | Success (picked, placed, complete, saved) |
| Yellow | Warning (stop, home) |
| Red | Error / failure |
| Blue | GUI-sent command |
| White | Informational |

## Camera Feed (right panel)

Subscribes to `/camera/annotated` (`sensor_msgs/Image`, `bgr8` encoding).
Displays the annotated frame from the OBB detector node at 336 × 252 px.
Shows "Waiting for camera..." until the first frame arrives.

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `F11` | Toggle fullscreen |
| `Escape` | Exit fullscreen |

## ROS2 Interface

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/motion_system/command` | `std_msgs/String` | All control commands (see above) |

### Subscribed

| Topic | Type | Published by |
|-------|------|-------------|
| `/system_status` | `std_msgs/String` | motion controller |
| `/robot_state` | `std_msgs/String` | motion controller |
| `/event_log` | `std_msgs/String` | motion controller |
| `/motion_system/status` | `std_msgs/String` | motion controller |
| `/motion_system/current_object` | `std_msgs/String` | motion controller |
| `/plastic_detections` | `std_msgs/String` (JSON) | OBB detector |
| `/camera/annotated` | `sensor_msgs/Image` | OBB detector |

## Dependencies

| Package | Purpose |
|---------|---------|
| `rclpy` | ROS2 Python client |
| `std_msgs`, `sensor_msgs` | ROS2 message types |
| `Pillow` (`PIL`) | BGR→RGB conversion and tkinter image display |
| `numpy` | Raw image buffer reshaping |
| `tkinter` | GUI framework (stdlib) |

Install Python deps if missing:
```bash
pip install Pillow numpy
```
