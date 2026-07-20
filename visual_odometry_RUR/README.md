# AutoSweep
## RUR53 Data Acquisition at Eureka Test Site

### Visual Odometry Tests

- cd /visual_odometry_RUR
- ./visual_odometry_acquisition_stack.py --camera-fps 15 --lidar-fps 10

Lidar+RGB+IMU:
- FAST-LIVO: https://github.com/hku-mars/FAST-LIVO
- LVI-SAM: https://github.com/TixiaoShan/LVI-SAM

Lidar-only:
- Genz-ICP: https://github.com/cocel-postech/genz-icp
- KISS-ICP: https://github.com/prbonn/kiss-icp
- LIO-SAM: https://github.com/TixiaoShan/LIO-SAM
- A-LOAM: https://github.com/HKUST-Aerial-Robotics/A-LOAM
- FAST-LIO2: https://github.com/hku-mars/fast_lio

Camera-only:
- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3