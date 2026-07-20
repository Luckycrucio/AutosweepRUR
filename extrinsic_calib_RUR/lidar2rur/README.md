# LiDAR-to-RUR motion calibration

`lidar_to_rur_handeye.py` estimates the rigid transform from a LiDAR frame
to the RUR odometry child frame using synchronized relative motions:

```text
A_base * T_base_lidar = T_base_lidar * B_lidar
```

The default inputs are:

```text
/odom             RUR wheel/base odometry
/genz/odometry    GenZ-ICP LiDAR-only odometry
```

Start GenZ without transforming its result into `base_link`; the LiDAR
trajectory must remain ego-centric to the LiDAR:

```bash
roslaunch genz_icp odometry.launch \
  topic:=/ousterDome/points \
  odom_frame:=genz_odom \
  base_frame:= \
  publish_odom_tf:=false
```

Then run the calibration:

```bash
python3 lidar_to_rur_handeye.py \
  --rur-topic /odom \
  --lidar-topic /genz/odometry \
  --base-frame base_footprint \
  --lidar-frame ousterDome/os_sensor \
  --initial-translation "0.30,0.00,0.55" \
  --initial-rpy-deg "0,0,0"
```

Drive slowly on a flat, high-friction surface. Include straight segments,
left and right turns, and combined translation/rotation. The node stops
automatically after 80 informative keyframes. Ctrl-C solves early after at
least 20 keyframes.

The default output is `lidar_to_rur.yaml` in this directory. In the default
planar mode, only x, y, and yaw are optimized. The supplied z, roll, and
pitch are preserved because flat-ground motion cannot reliably observe
them.

Use `--mode se3` only with genuine 6-DoF excitation. Always validate the
result on a separate trajectory before publishing it as a static TF.
