# README #

### CONNECTION TO THE RUR WITH AN EXTERNAL LAPTOP ###

**Install ROS noetic** on the external laptop used for the connection, following instructions at [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

**Clone the following repos** in your catkin workspace:

* rur_setup: ```git clone git@bitbucket.org:flexsight/rur_setup.git```
* rur_calibration: ```git clone git@bitbucket.org:flexsight/rur_calibration.git```


**Credentials** for connection to the RUR:  
    ```Username: summit```  
    ```Password: R0b0tn1K```  
    ```IP: 192.168.53.2```  

**Configure network** for the external laptop :

* create a wired network with ```static IP: 192.168.53.X``` and ```Netmask: 255.255.255.0```, being careful to **use as X any number from 10 to 20**
* connect the Ethernet cable to the LAN input on the RUR panel  
* check the connection: ```ping 192.168.53.2```

**Configure** the external laptop **as ROS slave**:

* in the ~/.bashrc, add:  
    ```alias activate_rur="export ROS_MASTER_URI=[http://192.168.53.2:11311/](http://192.168.53.2:11311/) && export ROS_IP=192.168.53.X"```    
    ```alias rur_ssh="ssh summit@192.168.53.2"```   
    ```alias rur_setup="sh $(rospack find rur_setup)/scripts/rur_setup.sh &"```
* check the connection to the RUR using ```rostopic list```. You should see all the topics published by the RUR. If more info is needed, check [http://wiki.ros.org/ROS/Tutorials/MultipleMachines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

**Synchronize** master and slave:

* install chrony on the external laptop: ```sudo apt-get install chrony```
* to configure the external laptop as time server, open with sudo /etc/chrony/chrony.conf and add these lines:  
    ```# make it serve time even if it is not synced (as it can't reach out)```  
    ```local stratum 8```  
    ```# allow the IP of the RUR to connect```  
    ```allow 192.168.53.2```  
* reboot the laptop  
* connect to the RUR via ssh and verify the synchronization  
    ```ssh summit@192.168.53.2``` 
    ```ntpdate -q 192.168.53.X```  
  The offset should be near to 0\. If it is still big, do:  
    ```sudo /etc/init.d/chrony restart```  
    ```ntpdate -q 192.168.53.X```  
  More info: [https://answers.ros.org/question/298821/tf-timeout-with-multiple-machines/](https://answers.ros.org/question/298821/tf-timeout-with-multiple-machines/)

**Connect** to the RUR:

* in a terminal, type (insert password above if requested):   
    ```activate_rur```  
    ```rur_ssh```  

### CONNECTION TO THE EXTERNAL SENSORS ###

* **Install Ouster** LiDAR **ROS drivers** by following [https://github.com/ouster-lidar/ouster-ros?tab=readme-ov-file](https://github.com/ouster-lidar/ouster-ros?tab=readme-ov-file)  
* **Install** **RealSense** camera **ROS drivers:**  
        ```sudo apt-get install ros-noetic-realsense2-camera```    
        ```sudo apt get install ros-noetic-realsense2-description```  
* **launch** everything needed to get data from sensors:  
    ```roslaunch realsense2_camera rs_d435_camera_with_model.launch```    
    ```roslaunch ouster_ros driver.launch sensor_hostname:=os-122220000120.local timestamp_mode:=TIME_FROM_ROS_TIME```  
* **Example** of a command to run a **data acquisition**:   
  ```rosbag record --topic /ouster/points /ouster/imu /scan_front /scan_rear /odom /merged_cloud /imu_data /imu/rpy/filtered /cmd_vel /camera/color/image_raw /camera/gyro/sample /camera/color/camera_info /camera/depth/camera_info /camera/depth/color_points /camera/accel/sample /tf /tf_static```  

### RUN RUR WITH CALIBRATION ###

* ```rur_setup```
* optional ```roslaunch rur_setup hdl_rur128.launch```

### SENSORS-RUR EXTRINSICS CALIBRATION ###

Extrinsics between Ouster LiDAR and RealSense camera are already calibrated, if sensors are not removed from their mounting support.    
To calibrate extrinsic parameters between these sensors and the reference frame of the RUR, follow this procedure:

* ```rur_setup```
* ```roslaunch rur_setup hdl_rur128.launch```
* record rosbag with /odom and /lidar_odom
* ```rosparam set use_sim_time true```
* ```rosrun rur_calibration odoms_to_txt.py``` (it will generate odoms.txt)
* ```rosrun rur_calibration odoms_aligner odoms.txt``` (it will print the output tx,ty,ry)
* manually copy tx,ty,tz in calibrated_tf.launch in rur_calibration package