### Sensors Drivers [INFO]

Run `sensors_drivers_stack.py` and select one of the three sensor combinations:

1. Two LiDARs (spinning and dome)
2. Azure Kinect and spinning LiDAR
3. Azure Kinect and dome LiDAR

The underlying driver commands are:

- roslaunch ouster_ros driver.launch sensor_hostname:=os-122220000120.local lidar_mode:=1024x10 point_type:=original
- roslaunch azure_kinect_ros_driver driver.launch fps:=15

After launching the selected drivers, the stack waits five seconds and then
starts the calibrated RUR53 TF/CAD publisher and RViz automatically. RViz uses
`base_footprint` as its fixed frame and loads
`RUR53/config/rviz.yaml`.

Supported rates:
- Azure Kinect: 5, 15, 30 fps
- Ouster OS1: 5, 10, 20 Hz  

Collect rosbag for the camera (launch the azure driver with a lower rate):

- rosbag record -O </destination/path> </topics>

### RUR STUFF ###
alias activate_rur="export ROS_MASTER_URI=http://192.168.53.2:11311/ && export ROS_IP=192.168.53.15"
alias rur_ssh="ssh summit@192.168.53.2"
alias rur_setup="$pp=$(pwd) & cd ~/Documents/wheelchair/ & source devel/setup.bash & sh src/rur_setup/scripts/rur_setup.sh & cd $pp"
