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

## Inputs / Outputs

### Published topics

| Topic | Type | QoS depth | Description |
|-------|------|-----------|-------------|
| `/motion_system/command` | `std_msgs/String` | 10 | All control commands (see Controls Reference) |

### Subscribed topics

| Topic | Type | QoS | Published by |
|-------|------|-----|-------------|
| `/system_status` | `std_msgs/String` | default | motion controller |
| `/robot_state` | `std_msgs/String` | default | motion controller |
| `/event_log` | `std_msgs/String` | default | motion controller |
| `/motion_system/status` | `std_msgs/String` | default | motion controller |
| `/motion_system/current_object` | `std_msgs/String` | default | motion controller |
| `/plastic_detections` | `std_msgs/String` (JSON array) | default | OBB detector |
| `/pick_queue` | `std_msgs/String` (JSON array) | default | workspace_planner_node |
| `/camera/annotated` | `sensor_msgs/Image` | BEST_EFFORT, depth=1 | OBB detector |

`/plastic_detections` is used as a fallback display source; once `/pick_queue` publishes its first message the GUI switches to that enriched view permanently for the session.

### Action clients

| Action server | Type | Used for |
|---------------|------|---------|
| `/gripper_action_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Open/Close buttons (direct, bypasses motion controller) |

### TF2

The GUI listens for the transform `base → gripper_tcp` when saving bin poses. Both `robot_state_publisher` and MoveIt must be running for this lookup to succeed.

### File I/O

| Path | Operation | Purpose |
|------|-----------|---------|
| `config/bin_poses.json` (source tree) | read on startup, write on save | Bin drop-off poses |
| `install/ur_gripper_demo/share/ur_gripper_demo/config/bin_poses.json` | write on save | Install-tree copy read by the motion controller at runtime |

Both paths are hardcoded absolutes (see [Configurable Settings](#configurable-settings--important-parameters)).

### Side effects

The **E-STOP** button does two things: publishes `estop` to `/motion_system/command` **and** calls `pkill -9 -f motion_controller_reactive`, which sends SIGKILL directly to the controller process.

---

## Configurable Settings / Important Parameters

All of the following are hardcoded constants at the top of [scripts/status_gui.py](scripts/status_gui.py). Change them in-source; there is no launch argument or ROS parameter interface.

| Constant | Default | Description |
|----------|---------|-------------|
| `BIN_POSES_PATH` | `/home/jarrel/.../src/.../config/bin_poses.json` | Source-tree path the GUI reads and writes |
| `BIN_POSES_INSTALL_PATH` | `/home/jarrel/.../install/.../config/bin_poses.json` | Install-tree path written simultaneously so the controller sees changes without a rebuild |
| `_BASE_FRAME` | `'base'` | TF2 world/base frame name for EEF lookups |
| `_EEF_FRAME` | `'gripper_tcp'` | TF2 end-effector frame name for EEF lookups |
| Gripper open position | `0.110` m (110 mm) | Target jaw separation for the Open button |
| Gripper close position | `0.0` m | Target jaw separation for the Close button |
| Gripper max effort | `40.0` N | Max force sent with every gripper action goal |
| TF2 lookup timeout | `1.0` s | How long `_get_eef_pose()` waits for the transform |
| ROS spin interval | `100` ms | `root.after(100, _spin_ros)` — caps callback frequency at ~10 Hz |
| Window initial size | `1380 × 920` px | `root.geometry(...)` — window is resizable after launch |
| Pick priority types | `pet_bottle`, `hdpe_bottle`, `pp_container` | Class names shown in the PICK PRIORITY radio group; must match OBB detector class names |

---

## Known Limitations and Assumptions

1. **Hardcoded absolute paths** — `BIN_POSES_PATH` and `BIN_POSES_INSTALL_PATH` are tied to `/home/jarrel/...`. The GUI will silently fail to save bin poses on any other machine or if the workspace is moved.

2. **10 Hz GUI update ceiling** — `rclpy.spin_once` is called every 100 ms. Camera frames and detections published faster than 10 Hz are silently dropped in the GUI, even though the ROS subscriber queue depth would allow buffering them.

3. **E-STOP is a hard kill** — `pkill -9` (SIGKILL) gives the motion controller no chance to execute a graceful shutdown (e.g., completing a safe stop trajectory). The arm stops wherever it is at the moment of the kill.

4. **Open/Close bypass the motion controller** — The Gripper buttons send directly to the action server, independently of the controller's internal state. Pressing Open or Close mid-pick can conflict with a grasp in progress.

5. **TF2 must be live to save bin poses** — If `robot_state_publisher` or MoveIt is not running, the TF2 lookup in `_get_eef_pose()` will time out and the save is aborted. The `SAVE_BIN` command is still sent to the controller, which may succeed on its own TF2 lookup.

6. **`/pick_queue` switching is one-way per session** — Once any message arrives on `/pick_queue`, the enriched display mode is locked in for the rest of the session (`_pick_queue_active` flag). If `workspace_planner_node` crashes, the pick queue display stalls rather than falling back to raw `/plastic_detections`.

7. **Camera encoding assumed BGR(A)** — The frame callback checks the number of channels but does not validate the `encoding` field of the image message. Images encoded as `rgb8` or `mono8` will display with wrong colours or crash the reshape.

8. **No namespace support** — All topic names are absolute. Running two instances of the GUI simultaneously will double-publish commands to the controller.

9. **`open_gripper` / `close_gripper` string commands are placeholder stubs** in the motion controller — the buttons send these strings to `/motion_system/command` but handlers must be implemented in `motion_controller_reactive.cpp` separately from the action-server path.

10. **FAILED state highlights in place, not at a distinct node** — When `robot_state` is `FAILED`, the current stepper node turns red rather than advancing to a dedicated FAILED position. The diagram stays visually at whichever stage failed.

---

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
