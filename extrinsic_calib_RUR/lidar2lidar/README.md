# LiDAR-to-LiDAR ICP calibration

`lidar2lidar_icp_calibration.py` estimates the fixed rigid transform between two
Ouster LiDARs from one synchronized point-cloud pair:

- target/reference: spinning LiDAR, `/ouster/points`
- source: dome LiDAR, `/ousterDome/points`

The output is `T_spinning_dome`, which maps dome points into the spinning-LiDAR
frame:

```text
p_spinning = T_spinning_dome * p_dome
```

In ROS TF terms, the spinning frame is the parent and the dome frame is the
child.

## Algorithm

The processing pipeline is:

```text
two PointCloud2 topics
        |
five valid approximately synchronized pairs
        |
range filtering, accumulation and voxel averaging
        |
initial mechanical transform guess
        |
coarse -> medium -> fine -> precision robust point-to-plane ICP
        |
transform matrix, quaternion, fitness and RMSE
        |
Open3D alignment visualization
```

### Synchronization and pair selection

ROS `ApproximateTimeSynchronizer` pairs messages whose header timestamps differ
by no more than `--sync-slop`, 0.06 seconds by default. This is tuned for
`1024x10`, where a new revolution is expected every 0.1 seconds. The algorithm
accepts the first five synchronized pairs for which both filtered clouds contain
at least `--min-points-per-cloud` points. Change the count with
`--collection-pairs`.

The scans are accumulated without odometry or motion compensation, so the scene
and sensor rig must remain stationary. Repeated measurements reduce random
range noise after voxel averaging. Moving objects can still create inconsistent
geometry.

### Filtering and downsampling

NaNs are discarded and points outside `--min-range` and `--max-range` are
removed. The defaults retain points from 1 to 60 metres. Very near points may
belong to the vehicle or sensor enclosure, while distant returns tend to be
noisier and consume more processing time.

Each accepted cloud is voxel-downsampled using `--collection-voxel`, 0.03 m by
default. A voxel grid divides space into cubes and retains one representative
point per occupied cube. This reduces memory, makes ICP faster, and prevents
very dense surfaces from dominating the optimization. After accumulation, a
second voxel pass averages repeated stationary samples in each occupied cube.

After this step, each cloud must contain at least 100 points and at least 2% of
its nominal `width * height` point count. This rejects extremely incomplete
clouds while allowing range filtering and the dome mounting to reduce the valid
return count. Increase `--min-valid-ratio` when the drivers publish reliably and
a stronger completeness check is desired.

### Initial transform

ICP is a local optimizer: it refines a plausible transform but cannot reliably
recover an arbitrary mounting from a poor initial guess. The initial rotation
uses roll, pitch and yaw in degrees:

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
T = [ R  t ]
    [ 0  1 ]
```

The default is `(roll=0, pitch=-90, yaw=+90)`, based on the measured mounting
of the front dome LiDAR relative to the horizontal spinning LiDAR. With the
rotation convention above, the resulting matrix is
`Rz(+90) * Ry(-90) * Rx(0)`. When applied to a point, the Y rotation acts first
and the Z rotation acts second.

Translation is specified in metres with `--initial-translation x,y,z`, where
the vector is the dome sensor origin expressed in the spinning frame. Measure
this displacement on the rig. Zero translation is only a placeholder and can
cause ICP to converge incorrectly when overlap is limited.

### Multi-scale ICP

ICP repeatedly transforms the dome cloud, finds nearby points in the spinning
cloud, and updates the six-degree-of-freedom rigid transform to reduce alignment
error. A single fine-resolution ICP has a small capture range, so this script
runs four stages and passes each result to the next:

| Stage | Voxel size | Maximum correspondence distance | Iterations |
|---|---:|---:|---:|
| Coarse | 0.50 m | 1.50 m | 80 |
| Medium | 0.25 m | 0.75 m | 60 |
| Fine | 0.10 m | 0.30 m | 50 |
| Precision | 0.05 m | 0.12 m | 40 |

The coarse stage retains large structures and tolerates more initial error. The
later stages progressively introduce detail and reject more distant matches.
The equally sized lists can be changed with `--voxel-sizes`,
`--max-correspondence`, and `--iterations`. Correspondence distances should
normally be larger than their corresponding voxel sizes.

### Point-to-plane error

At each scale, local surface normals are estimated with a search radius equal
to three times the voxel size. ICP conceptually minimizes:

```text
sum_i ((R * p_i + t - q_i) dot n_i)^2
```

Here `p_i` is a dome point, `q_i` is its corresponding spinning-LiDAR point,
and `n_i` is the target surface normal. Point-to-plane ICP normally converges
faster than point-to-point ICP on ground, walls, facades and other planar
surfaces. A Huber robust loss at each scale reduces the influence of residual
outliers, and stricter convergence tolerances retain small final updates. The
scene must contain surfaces in several orientations: flat ground alone cannot
constrain every translation and rotation component.

## Running

Source ROS Noetic and the workspace, then run:

```bash
python3 extrinsic_calib_RUR/lidar2lidar/lidar2lidar_icp_calibration.py \
  --initial-rpy-deg 0,-90,90 \
  --initial-translation 0,0,0
```

Use a measured translation in place of `0,0,0`, for example:

```bash
python3 extrinsic_calib_RUR/lidar2lidar/lidar2lidar_icp_calibration.py \
  --initial-rpy-deg 0,-90,90 \
  --initial-translation 0.25,0,-0.35
```

The translation above is only an example.

Useful options:

- `--sync-slop 0.06`: timestamp tolerance tuned for 10 Hz input
- `--expected-hz 10`: expected topic rate shown by diagnostics
- `--expected-width 1024`: expected cloud width shown by diagnostics
- `--min-range 1.0 --max-range 60.0`: accepted radial range
- `--collection-voxel 0.03`: input-cloud downsampling and averaging
- `--collection-pairs 5`: stationary synchronized pairs to aggregate
- `--min-points-per-cloud 100`: minimum usable size after preprocessing
- `--min-valid-ratio 0.02`: processed points required versus nominal size
- `--voxel-sizes 0.50,0.25,0.10,0.05`: ICP resolutions
- `--max-correspondence 1.50,0.75,0.30,0.12`: correspondence gates
- `--iterations 80,60,50,40`: maximum iterations per scale
- `--output path.yaml`: output location
- `--publish-tf`: publish the result and keep the node alive
- `--no-visualization`: skip the viewer when running without a desktop

Frame names come from the `PointCloud2` headers. ROS private parameters
`~spinning_frame` and `~dome_frame` can override them.

## Alignment visualization

Before ICP starts, the script opens an initial-guess editor containing the
fixed spinning cloud in gray and the dome cloud in blue. The view is fitted to
both clouds. Its controls provide:

- roll, pitch and yaw sliders from -180 to +180 degrees;
- dedicated -90 and +90 degree buttons for each X, Y and Z rotation;
- X, Y and Z translation sliders from -10 to +10 metres;
- a **Use guess and run ICP** button.

The values from `--initial-rpy-deg` and `--initial-translation` seed these
controls. The transform visible when the editor is accepted is passed directly
as the first initial guess to multi-scale ICP. Closing the editor also accepts
its current values. `--no-visualization` skips the editor and uses the
command-line values directly.

After saving the result, the script opens one final Open3D panel containing:

- the unchanged spinning cloud in gray;
- the dome cloud transformed by the final ICP `T_spinning_dome` in orange;
- RGB axes for the unchanged spinning frame;
- RGB axes for the dome frame placed by the final ICP transform.

Both clouds and frames are displayed in the spinning-LiDAR coordinate frame.
The initial dome cloud and initial dome frame are not included in the final
panel. Older Open3D versions fall back to a single legacy window.

Rotate the view by dragging with the mouse, zoom with the scroll wheel, and
close the window when inspection is complete. Shared surfaces in the orange
dome cloud and gray spinning cloud should coincide, particularly ground, walls,
corners and poles. Parallel but separated surfaces usually indicate a
translation error; an error that grows with distance usually indicates a
rotation error.

For every synchronized input pair, the terminal reports each cloud's frame,
width, height, nominal point count, serialized payload size in MiB, and number
of points remaining after range filtering and collection-voxel downsampling.

The viewer requires a graphical desktop and an OpenGL display. Use
`--no-visualization` for a headless SSH session. When `--publish-tf` is also
given, static-TF publication starts after the visualization window is closed.

## Output and validation

The default output, `dome_to_spinning.yaml`, contains:

- the 4x4 dome-to-spinning matrix;
- translation in metres and quaternion in `x,y,z,w` order;
- source/target topics and parent/child frames;
- final ICP fitness and inlier RMSE;
- `input_pairs`, documenting the number of aggregated synchronized pairs.

Fitness is the fraction of source points with a correspondence inside the final
distance threshold. Inlier RMSE is the root-mean-square residual of those
accepted correspondences. Higher fitness and lower RMSE are generally better,
but neither proves that the alignment is correct. Sparse overlap or repetitive
geometry can produce a convincing metric for the wrong transform.

Validate every result in RViz using the spinning frame as the fixed frame and
`--publish-tf`. Inspect ground, walls, corners, poles and nearby objects. Repeat
the calibration with several separate pairs and compare the resulting poses;
large variation indicates weak geometry, motion, poor overlap, timestamp
problems, incomplete clouds, or an inaccurate initial guess.

Choose a stationary scene containing geometry at different depths and in
several orientations. Avoid moving vehicles and pedestrians, vegetation moving
in wind, a flat road with no other structure, and highly repetitive corridors.

## Input troubleshooting

While waiting, the node reports message counts, publisher connections,
synchronized-pair count, and the latest timestamp difference every five
seconds. Check the streams independently with:

```bash
rostopic type /ouster/points
rostopic type /ousterDome/points
rostopic hz /ouster/points
rostopic hz /ousterDome/points
rostopic echo -n 1 /ouster/points/header
rostopic echo -n 1 /ousterDome/points/header
```

Both types must be `sensor_msgs/PointCloud2`. If both message counters increase
but no synchronized pair appears, increase `--sync-slop` or verify that both
drivers use the same clock. If pairs are repeatedly rejected for too few
points, fix packet loss or driver publication before lowering the threshold.

## Limitations

- ICP needs sufficient common geometry and a reasonable initial transform.
- The method uses only one pair and does not average measurement noise.
- It does not deskew scans or compensate for sensor/scene motion.
- It does not estimate clock offset or intrinsic LiDAR calibration.
- Dynamic, symmetric, repetitive, or mostly planar scenes can produce an
  incorrect or weakly constrained result.

Dependencies are ROS Noetic Python packages plus `numpy`, `PyYAML`, and
`open3d`.
