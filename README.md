Kinematic sliding controller for differential robots in ROS2.

In order to use:
- run the rider1.cpp file (ros2 run rider1 after sourcing install/setup.bash)
- publish Bool type message to /robot/start in order to start the movement (ros2 topic pub -1 /robot/start std_msgs/Bool "{data: true}") 

All the necessary information about the trajectory should be included in the trajectory.json file in resources/ folder. The trajectory should contain the following information: 
'x'-x coordinate, 
'y'-y coordinate, 
'0'-robot orientation, 
'v'-reference linear velocity, 
'w'-reference angular velocity, 't'-time in [s].

Example trajectory file is in the /resources folder.

The 'rider1.cpp' publishes to the /robot0 topic, e.g. /robot0/cmd_vel, as this controller is intended for use in cooperative transportation tasks. In order to use with simple turtlebot, change the corresponding topics to defaults, e.g. /cmd_vel.

The Controller constructor takes two arguments, which are k1 and k2 values. These values control the behavior of the controller. Each robot has a unique set of values, but from my experience the k1=.2 and k2=.2 should work well with turtlebot burger model.

If you want to spawn a burger turtlebot with remapped topics (/robot0 etc.), the two_robot.launch.py script should be placed in the dev/install/<this_package_name>/share/<this_package_name>/. After sourcing, use the script with 'ros2 launch two_robot.launch.py'.

This controller is based on the following papers:
- "Sliding Mode Control for Trajectory Tracking of Mobile Robot in the RFID Sensor Space", Jun Ho Lee, Cong Lin, Hoon Lim, and Jang Myung Lee, International Journal of Control, Automation, and Systems (2009) 7(3):429-435
- "Formation-based Control Scheme for Cooperative Transportation by Multiple Mobile Robots", Alpaslan Yufka and Metin Ozkan, International Journal of Advanced Robotic Systems
