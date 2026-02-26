# Robotics-Studio2
For simulation connecting with move it freedrive

Start by running [ros2 run ur_client_library start_ursim.sh -m ur3e],
This is to connect the polyscope, connect with [http://192.168.56.101:6080/vnc.html],

Run the controller to simulator ip [ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true],

The ip is the ip for the simulator,

Then run the move it [rviz ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true],

------------------------------------------------

For Real Robot moving

Start by turning on the power and connecting the robot,

Get the ip of your laptop my case (192.168.0.193), --> for both ip and host name,
Connect it on the teach pendant --> Program --> URCAps --> TCP --> Control by --> type in hostname and ip,

Run controller with robot id [ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.197 launch_rviz:=true],

Click the play button on the bottom right and click "Play from selection",

Run the rviz move it [ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true],

------------------------------------------------

To get ip --> hostname -I --> your ip is the last one starting with 192

Ping Robot --> ping 192.168.0.197 --> you are just checking network connectivity.