export ROS_IP=192.168.100.150
export ROS_MASTER_URI=http://192.168.100.183:11311
source /opt/ros/melodic/setup.bash
source ~/ros/devel/setup.bash
roslaunch pci_boards_driver cpz7415v.launch
