# ROS2 and ROS1 based Radar drivers on C++
Used for SLAM (Simultaneous Localization and Mapping) with 3D RBD-D (Oak-D Pro Camera) x 4D Altos Radar for Volumteric Mapping with Mobile Robots.

# ROS2 REAL-TIME SETUP: Copying Radar drivers to/inside Robot 
1. SSH to Robot i.e. ubuntu@192.168.186.3
2. Enter Password i.e. robotname
3. scp -r ~/robohub/turtlebot/altosRadarROS2 ubuntu@192.168.186.3:/home/ubuntu/
4. sudo ip addr add 192.168.3.1/24 dev eth0
(Above is to read the ethernet cable attached to controller of robot)
5. ros2 run altosradar altosRadarParse

# ROS2 bag convert to ROS1
rosbags-convert --src rosbag2_2024_09_19-14_25_19 --dst ./rosbag2_2024_09_19-14_25_19_ros.bag

# altosRadar ROS1 Setup
1. git clone https://github.com/Altos-Radar/altosRadarRos.git
2. cd altosRadarRos
3. catkin_make
4. open terminal in altosRadarRos
   roscore
5. open terminal in altosRadarRos
   rosrun rviz rviz
   open File-->Open Config-->choose altosRadarRos/src/altosradar/altosradar.rviz
6. open terminal in altosRadarRos
   source devel/setup.bash
   rosrun altosradar altosRadarParse  
   
------------------------------------------------------------------------------------   
   
   
OR
1. git clone https://github.com/Altos-Radar/altosRadarRos.git
2. cd altosRadarRos
3. catkin_make
4. open terminal in altosRadarRos
   ./start.sh
   
  rosbag replay: 
  1. catkin_make
  2. source devel/setup.bash
  3. roslaunch rviz.launch
  4. rosbag play xxx.bag
