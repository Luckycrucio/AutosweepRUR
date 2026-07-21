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

## How the algorithm works

The calibration treats the rigid mounting transform between the LiDAR and
robot base as constant. It does not compare the absolute poses published by
the two odometry systems, because their world frames and initial poses can be
different. Instead, it compares how both sensors report the robot moving over
the same time intervals.

The processing pipeline is:

1. **Synchronize odometry.** The node uses approximate timestamp
   synchronization to associate each `/odom` pose with a `/genz/odometry`
   pose. `--sync-slop` sets the maximum permitted timestamp difference and
   `--sync-queue` controls the synchronization queue. The timestamp deltas are
   retained and summarized in the output YAML.

2. **Select informative keyframes.** The first synchronized pose is accepted.
   A later pose is retained only when the motion since the last accepted
   keyframe exceeds either `--min-translation` or `--min-rotation-deg` in at
   least one trajectory. This avoids filling the problem with nearly identical
   stationary measurements. Collection ends at `--samples`, or can be stopped
   with Ctrl-C once `--min-samples` have been collected.

3. **Construct relative-motion pairs.** For each configured index separation
   in `--pair-gaps` (default `1,2,4,8`), the algorithm computes the relative
   base motion `A_base` and relative LiDAR motion `B_lidar` between two
   synchronized keyframes. Using several gaps provides both short- and
   longer-baseline constraints. With the unknown mounting transform
   `T_base_lidar`, every pair should satisfy:

   ```text
   A_base * T_base_lidar = T_base_lidar * B_lidar
   ```

4. **Optimize the mounting transform.** The residual measures the translation
   and rotation disagreement between the two sides of that equation for every
   motion pair. SciPy's nonlinear least-squares solver minimizes all residuals
   together. The default `soft_l1` loss reduces the influence of occasional
   odometry outliers; `--loss` selects another robust loss and
   `--rotation-weight` controls the relative importance of rotational error.

5. **Handle planar observability.** In the default `planar` mode, each motion
   is projected onto the ground plane and only LiDAR-to-base x, y, and yaw are
   estimated. Because yaw can have distant local minima, the solver tries 12
   evenly spaced initial yaw values and retains the solution with the smallest
   residual. The terminal status bar reports completion of these 12 solves.
   The z, roll, and pitch components are copied from `--initial-translation`
   and `--initial-rpy-deg`; ordinary flat-ground driving cannot determine them.

6. **Save and diagnose the result.** The selected transform is written in
   translation/RPY, quaternion, and 4x4 matrix forms. The YAML also records
   optimizer convergence, translation and rotation residual RMSE, Jacobian
   singular values and condition number, synchronization quality, the number
   of keyframes and relative-motion pairs, and motion statistics. A successful
   optimizer termination means the numerical solve converged; accuracy must
   still be judged from residuals, conditioning, and validation data.

The reported direction is `lidar_to_base`: the result maps coordinates from
the LiDAR child frame into the robot base parent frame. In ROS TF terms, the
base is the parent and the LiDAR is the child.

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
