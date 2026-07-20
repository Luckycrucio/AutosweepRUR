# AutoSweep
## RUR53 Data Acquisition at Eureka Test Site

### Sensors Drivers

- Rosbag Acquisition:/sensors_drivers_stack.py
    - ./autosweep/extrinsic_calib_RUR/sensors_drivers_stack.py 

#### Two-LiDAR timing

In two-LiDAR mode, the two driver terminals are launched immediately back to
back, followed by one five-second initialization wait. Both drivers use
`TIME_FROM_ROS_TIME`, so their messages share the ROS time domain. Back-to-back
launching does not provide hardware scan synchronization, but it avoids an
unnecessary software startup delay and is suitable for approximate pairing in
a stationary calibration scene.

The same startup pattern is used in LiDAR-camera mode: the Ouster and Azure
Kinect terminals are launched immediately back to back, and the program waits
five seconds only after both drivers have been started.

The two-LiDAR launch assigns distinct UDP destinations on the acquisition
computer. The spinning LiDAR uses ports 7502/7503 for LiDAR/IMU packets, and the
dome LiDAR uses 7504/7505. ROS namespaces only separate ROS resources; they do
not separate UDP sockets, so using explicit non-overlapping ports makes the two
packet paths deterministic. LiDAR-camera mode uses 7502/7503 for its one Ouster.

Before launching any sensor terminal, the script checks `/run_id` on the ROS
parameter server. If no ROS master is available, it starts one `roscore` and
waits until it is ready. This prevents the back-to-back `roslaunch` processes
from racing to create separate masters, which otherwise produces a mismatched
`run_id` and a port-11311 error. An already-running ROS master is reused and is
not stopped by the script; a master created by the script is stopped on exit.


### Extrinsic Calibration


### CA2LIB

Collect measurements:

- rosrun ca2lib calibrate_lidar_camera \
    -c /rgb/image_raw \
    -i autosweep/extrinsic_calib_RUR/lidar2rgb/config/cam_intrinsics.yaml \
    -l /ouster/points \
    -t autosweep/extrinsic_calib_RUR/lidar2rgb/config/chessboard_target.yaml  \
    --output-planes autosweep/extrinsic_calib_RUR/lidar2rgb/measures/measures4.txt \
    -a

Perform offline optimization:

- rosrun ca2lib calibrate_lidar_camera_offline \
    --input autosweep/extrinsic_calib_RUR/lidar2rgb/measures/measures2.txt \
    --output autosweep/extrinsic_calib_RUR/lidar2rgb/lidar_in_cam_T/lidar_in_cam2.yaml \
    --iterations <num_iter> \
    --threshold-inlier <inlier_th> \
    --huber <huber_th> \
    --damping <damping_factor>

### KENJI KOIDE

- Koide Extrinsic Calibration:
    - Preprocessing: 
        - rosrun direct_visual_lidar_calibration preprocess ../../media/fsd/3127552a-0703-402f-ad8e-991656cc348c/autosweep/extrinsic/currentBags ouster_preprocessed  -a -d -v
    - Initial guess (Manual):
        - rosrun direct_visual_lidar_calibration initial_guess_manual ouster_preprocessed
    - Initial guess (Automatic)
        - rosrun direct_visual_lidar_calibration find_matches_superglue.py ouster_preprocessed
        - rosrun direct_visual_lidar_calibration initial_guess_auto ouster_preprocessed
    - Fine Registration:
        - rosrun direct_visual_lidar_calibration calibrate ouster_preprocessed
    - Validation:
        - python3 autosweep/extrinsic_calib_RUR/validation.py \
            --mode koide \
            --folder outdoorJun24
        with new intrinsics :
            - python3 validation.py --bag outdoorJun24NewInt/bags/outdoorScene1.bag --calib outdoorJun24/ouster_preprocessed/calib.json \
                --K 969.0834729470149,968.6962843734808,1037.720183334357,785.6266408743057 \
                --D 0.09669489301600527,-0.08214255351820926,-0.0004322600706973199,-0.00007702140183761972,0.03952446125336604


### Validation
    - python3 autosweep/extrinsic_calib_RUR/backprojective_validation.py \
        --mode ca2lib \
        --folder outdoorJun24 \
        --yaml autosweep/extrinsic_calib_RUR/lidar2rgb/lidar_in_cam_T/lidar_in_cam_3_tuned.yaml
