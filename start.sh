source install/setup.bash
gnome-terminal --tab -t "rviz" -- bash -c "ros2 launch altosradar rviz_launch.py; read"
sleep 3s
gnome-terminal --tab -t "altosradar" -- bash -c "ros2 run altosradar altosRadarParse; read"

