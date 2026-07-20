### Sensors Drivers [INFO]

Launch ouster OS1 and Azure drivers:

- roslaunch ouster_ros driver.launch sensor_hostname:=os-122220000120.local lidar_mode:=1024x10 point_type:=original
- roslaunch azure_kinect_ros_driver driver.launch fps:=15

Supported rates:
- Azure Kinect: 5, 15, 30 fps
- Ouster OS1: 5, 10, 20 Hz  

Collect rosbag for the camera (launch the azure driver with a lower rate):

- rosbag record -O </destination/path> </topics>

### RUR STUFF ###
alias activate_rur="export ROS_MASTER_URI=http://192.168.53.2:11311/ && export ROS_IP=192.168.53.15"
alias rur_ssh="ssh summit@192.168.53.2"
alias rur_setup="$pp=$(pwd) & cd ~/Documents/wheelchair/ & source devel/setup.bash & sh src/rur_setup/scripts/rur_setup.sh & cd $pp"
