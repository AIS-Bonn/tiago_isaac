## Colored RGB PointCloud Visualization

This workspace contains custom ROS2 launch files.

### Functionality
- `src/tiago_rviz2/launch/rgb_pointcloud.py`:
  - Takes the depth and RGB images from an RGB-D camera.
  - Converts them into a colored point cloud.
  - Publishes the resulting point cloud to the topic `gemini2/rgbpoints`.
  - Allows visualization of the colored point cloud using the PointCloud2 display in RViz2.

### Building and Running the Workspace

Execute the following commands in your terminal:

```bash
cd ros2_ws/
colcon build
cd ..
source ros2_ws/install/setup.bash
ros2 launch tiago_rviz2 rgb_pointcloud.py
```

### Visualization in RViz2

- Open RViz2.
- Add a `PointCloud2` display.
- Set the topic to `gemini2/rgbpoints` to visualize the generated colored point cloud.

