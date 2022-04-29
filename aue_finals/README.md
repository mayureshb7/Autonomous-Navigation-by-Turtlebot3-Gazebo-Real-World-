This file describes running Autonomous Navigation by Turtlebot3 Burger on Gazebo and Real world

	1) Autonomous Navigation - Gazebo
		a) Run command: "roslaunch aue_finals turtlebot3_autonomy_final.launch" to successfully launch world file in gazebo simulation
		b) Run command: "roslaunch aue_finals continuous_detection_gazebo.launch " for execution of april tag detection in gazebo simulation
		
		
	2) Autonomous Navigation - Real World
		a) Run Roscore on the pc terminal
		b) Bring up turtlebot3 burger by using this command on turtlebot terminal: roslaunch turtlebot3_bringup turtlebot3_robot.launch
		c) Bring up raspi camera by using this command on turtlebot terminal: roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
		d) Compress the raspi camera image feedback by using this command on the PC:  rosrun image_transport republish compressed in:=camera/image raw out:=camera/image_rae
		e) Run Command rqt_image_view to view camera
		f) Run Command: "roslaunch aue_finals continuous_detection.launch" for execution of april tag detection in real world
		g) Run Command: "roslaunch darknet_ros darknet_ros.launch" for execution of yolo sign detection in real worls (Launches Tiny Yolo)
		h) Run following command on PC terminal to execute the Autonomous Navigation code on actual bot: rosrun aue_finals wall_follower_line_following_Line_real_World.py
