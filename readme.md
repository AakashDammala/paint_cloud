# Paint Cloud

A ROS 2 package for generating robot manipulator painting paths from 3D perception data.

This package scans the environment using a depth camera (simulated in Gazebo), computes surface normals from the point cloud, and generates a coverage path (zig-zag pattern) for a painting end-effector. The path conforms to the surface curvature and aligns the end-effector orientation with the surface normals.

## Features

- **PointCloud Processing**: Downsamples and transforms raw point cloud data from the camera frame to the world frame.
- **Surface Normal Estimation**: Uses Open3D to estimate normals for every point in the cloud.
- **Coverage Path Planning**: Generates a zig-zag path based on configurable surface dimensions, brush radius, and overlap factor.
- **3D Projection**: Uses a KD-Tree to project the 2D planned path onto the actual 3D curved surface.
- **Orientation Alignment**: Computes the orientation for each waypont to align the tool with the surface normal.
- **Service Interface**: Provides a ROS 2 service to trigger path generation and retrieve the path.
- **Visualization**: Publishes markers (normals), point clouds, and path poses for visualization in RViz.

## Installation

### Dependencies

- ROS 2 (humble/jazzy)
- `ros_gz` (ROS 2 - Gazebo Bridge)
- **Python Libraries**:
  - `open3d`
  - `scipy`
  - `numpy`
  - `transforms3d` (or `tf_transformations`)

### Building

1. Create a workspace (if you haven't already):
   ```bash
   mkdir -p ~/paint_ws
   cd ~/paint_ws
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/AakashDammala/paint_cloud.git src/
   ```

3. Install dependencies:
   ```bash
   cd ~/paint_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
   *Note: If you have to manually install python packages:*
   ```bash
   pip3 install open3d scipy
   ```

4. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Usage

### Launching the Simulation

This launches Ignition Gazebo with a RealSense camera environment, the ROS-Ignition bridge, RViz2, and the `paint_cloud` node.

```bash
ros2 launch paint_cloud gazebo.launch.py
```

### Triggering Path Generation

The node starts automatically but waits for a service call to generate and return the path.

```bash
ros2 service call /get_paint_path paint_cloud_msgs/srv/GetPaintPath {}
```

### Parameters

You can configure the following parameters in `paint_cloud.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `downsample_factor` | `100` | Factor to downsample the input point cloud, higher the factor, less dense the point cloud. |
| `brush_radius` | `0.1` | Radius of the painting brush (meters). |
| `overlap_factor` | `1.0` | Overlap between painting strokes (1.0 - no overlap, 0.5 - half overlap). |
| `surface_length_x` | `1.0` | Length of the target surface area (meters). |
| `surface_width_y` | `0.8` | Width of the target surface area (meters). |

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/realsense/points` | `sensor_msgs/PointCloud2` | Input point cloud from the camera. |
| `curved_cloud` | `sensor_msgs/PointCloud2` | Processed point cloud in world frame. |
| `surface_normals` | `visualization_msgs/MarkerArray` | Visual markers for surface normals. |
| `paint_path` | `geometry_msgs/PoseArray` | The generated coverage path with orientations. |
| `surface_polygon` | `geometry_msgs/PolygonStamped` | Visualization of the target surface boundaries. |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `get_paint_path` | `paint_cloud_msgs/GetPaintPath` | Triggers path generation and returns the path poses. |