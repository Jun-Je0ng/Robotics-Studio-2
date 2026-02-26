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

5. Start the driver(Controller) (robot IP example: 192.168.0.197):
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