# RUR53 calibrated RViz visualization

This visualization uses `base_footprint` as the fixed frame and publishes the
following calibrated static tree:

```text
base_footprint
└── os_sensor                        lidar_to_rur.yaml (+0.443 m Z)
    ├── os_lidar                     spinning Ouster driver
    ├── os_imu                       spinning Ouster driver
    ├── ousterDome/os_sensor         dome_to_spinning.yaml
    │   ├── ousterDome/os_lidar      dome Ouster driver
    │   └── ousterDome/os_imu        dome Ouster driver
    └── camera_base                  lidar_in_cam_3_tuned.yaml + Azure TF
        └── ...
            └── rgb_camera_link      Azure driver
```

The RUR53 CAD mesh is authored in millimetres. Its x/y coordinates are already
centered at zero; the marker publisher scales it by `0.001` and applies
`x = -0.223 m` and `z = -0.255 m`. The X offset moves the model backward
relative to `base_footprint`; the Z offset places the midpoint of the requested
510 mm reference height (`510 / 2 = 255 mm`) at the frame's vertical origin.

Start the sensor drivers and a ROS master, then run:

```bash
python3 RUR53/config/publish_rur53_visualization.py --include-camera
```

In another sourced terminal, launch RViz:

```bash
rviz -d RUR53/config/rviz.yaml
```

The publisher reads all three calibration YAML files at startup, so rerunning
it picks up later calibration results without copying transform values into the
RViz configuration. Use `--camera-frame` if the Azure driver reports a
different RGB frame ID.

`os_sensor` is the canonical spinning-LiDAR sensor frame. The Ouster driver
publishes its internal `os_lidar` and `os_imu` children. The visualization
publisher supplies `base_footprint -> os_sensor`, which is the link RViz needs
to transform `/ouster/points` while retaining `base_footprint` as the fixed
global frame. The dome driver uses the `ousterDome` prefix to keep its internal
frame names unique.

Do not run another static publisher for `os_sensor`,
`ousterDome/os_sensor`, or `camera_base` at the same time; every TF child must
have exactly one parent. The visualization publisher deliberately attaches
`camera_base`, not `rgb_camera_link`, so the Azure driver remains the sole
publisher of its internal camera frames.
