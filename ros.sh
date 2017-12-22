gnome-terminal -e "roscore"
sleep 2
gnome-terminal -e "roslaunch openni_launch openni.launch depth_registration:=true"
sleep 2
gnome-terminal -e "roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" "
gnome-terminal -e "rosrun rviz rviz"
gnome-terminal -e "rosrun rosserial_python serial_node.py /dev/ttyUSB0"
gnome-terminal -e "rosrun Navigator Navigator"

