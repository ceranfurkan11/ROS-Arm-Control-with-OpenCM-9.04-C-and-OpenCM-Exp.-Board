Firstly you need create package with '$catkin_create_package robot_arm_control std_msgs rospy roscpp geometry_msgs' in your workspace.
Then enter this package. Write '$git clone https://github.com/ceranfurkan11/ROS-Arm-Control-with-OpenCM-9.04-C-and-OpenCM-Exp.-Board.git'.
Then '$catkin_make' in your workspace.

1- Upload arduino code to OpenCM 9.04.
2- Make connections between ROS and OpenCM by serial node: '$rosrun rosserial_server serial_node _port:=/dev/ttyACM0'.
3- Launch Gazebo with '$roslaunch robot_arm_control robot_arm_position_control.launch'.
4- Run the codes of you want. For example, '$rosrun robot_arm_control sliding_mode.py'.
