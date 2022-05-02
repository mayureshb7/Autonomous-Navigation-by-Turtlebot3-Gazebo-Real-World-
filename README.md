**About Project**

This Project is about running Autonomous Navigation by Turtlebot3 Burger on Gazebo and Real world as a part of course AuE 8230 Autonomy Systems and Sciences

**Commands to run Gazebo Simulation**

	1) Autonomous Navigation - Gazebo
		a) Run command to successfully launch world file in gazebo simulation
			roslaunch aue_finals turtlebot3_autonomy_final.launch

		b) Run following command for execution of april tag detection in gazebo simulation
			roslaunch aue_finals continuous_detection_gazebo.launch

**Experiment Video - Gazebo**
https://github.com/mayureshb7/AuE8230_Turtlebot3_Autonomous/blob/master/aue_finals/Autonomous-Turtlebot-Gazebo.mp4

![Autonomous_Turtlebot_Gazebo](https://user-images.githubusercontent.com/99101076/166168122-377c6f50-f067-4db2-ac80-77a31ed547d7.gif)

**Commands to run Real World Simulation**

	2) Autonomous Navigation - Real World
		a) Run Roscore on the pc terminal
		b) Bring up turtlebot3 burger by using this command on turtlebot terminal: roslaunch turtlebot3_bringup turtlebot3_robot.launch
		c) Bring up raspi camera by using this command on turtlebot terminal: roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
		d) Compress the raspi camera image feedback by using this command on the PC:  rosrun image_transport republish compressed in:=camera/image raw out:=camera/image_rae
		e) Run Command rqt_image_view to view camera
		f) Run Command: "roslaunch aue_finals continuous_detection.launch" for execution of april tag detection in real world
		g) Run Command: "roslaunch darknet_ros darknet_ros.launch" for execution of yolo sign detection in real worls (Launches Tiny Yolo)
		h) Run following command on PC terminal to execute the Autonomous Navigation code on actual bot: rosrun aue_finals wall_follower_line_following_Line_real_World.py

**Experiment Video - Real World**

**Wall Following, Obstacle Avoidance and Line Following**

![Wallfollowing_Obstacleavoidance_Linefollowing](https://user-images.githubusercontent.com/99101076/166171033-ea3a4f62-f86a-43a5-a636-fe2fa50b887b.gif)

**Stop Sign Detection**

![Stop_Sign_Detection](https://user-images.githubusercontent.com/99101076/166171044-4a446710-c5ad-4bf6-8f32-76bea5922487.gif)

**April Tag Following**

![April_Tag_Following](https://user-images.githubusercontent.com/99101076/166171053-29f98ed0-dfb6-4ab9-a1c6-3fe482091eb7.gif)
