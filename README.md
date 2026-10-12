# skyplexus
A ROS2-based UAV high-resolution sensing testbed, with LiDAR, radar, camera, RTK GPS, and Cube Orange running ArduPilot.

A PyTorch node fuses the LiDAR and camera into a colorized 2.5D elevation map on the GPU.

Personal project, built solo during the winter of my senior year of high school (2024–25).

![Demo](media/drone.gif)

Flight over my house and the surrounding street. Flight data is not included in this repository.

![Uncolored map in RViz](media/rviz_map.png)

Uncolored version of the map, viewed in RViz.

## Architecture

```mermaid
flowchart TD
    subgraph onboard["Onboard: Jetson · ROS 2 Humble · Isaac ROS container"]
        lidar["Ouster OS1-64<br/>PTP-synced"]
        cam["Orbbec Gemini 330<br/>RGB-D"]
        fc["Cube Orange · ArduPilot<br/>RTK GPS + IMU"]
        radar["Altos 4D radar"]
        bag[("rosbag2")]
        lidar -->|"/ouster/points"| bag
        cam -->|"/camera/color/image_raw"| bag
        fc -->|"MAVROS → /tf"| bag
        radar -.->|"/altos/altosRadar<br/>(recorded, not used by map_gen)"| bag
    end
    subgraph post["Post-flight mapping"]
        clean["clean_bag.py<br/>drop bad poses + /tf_static"]
        launch["mapgen.launch.py<br/>static TF tree + bag replay"]
        mg["map_gen.py<br/>PyTorch · CUDA"]
        rviz["RViz"]
        clean --> launch --> mg -->|"/pcl · XYZRGB 2.5D grid"| rviz
    end
    bag --> clean
```

## TF tree

```mermaid
flowchart TD
    map -->|"static · grid origin"| home
    map -.->|"MAVROS · ArduPilot EKF<br/>(RTK GPS + IMU)"| base_link
    base_link -->|"x 0.10 · z −0.42 · pitch 45°"| payload
    payload -->|"roll 180° · LiDAR inverted"| os_sensor
    payload -->|"x 0.045 · z 0.22"| camera_link
    camera_link --> camera_depth_frame
    camera_depth_frame -->|"Orbbec factory extrinsic"| camera_color_frame
    camera_color_frame -->|"optical: z fwd, y down"| camera_color_optical_frame

    classDef used stroke:#f59f00,stroke-width:3px
    class home,os_sensor,camera_color_optical_frame used
```

Solid edges are static transforms from `mapgen.launch.py`; the dashed edge is published by MAVROS. Highlighted frames are the ones `map_gen` looks up: `os_sensor → camera_color_optical_frame` and `os_sensor → home`.

## How map_gen works

All tensors stay on the GPU.

1. Keeps the latest camera frame from `/camera/color/image_raw`.
2. For each LiDAR scan on `/ouster/points`, looks up `os_sensor → camera_color_optical_frame` and `os_sensor → home`.
3. Projects the points through the camera intrinsics and keeps the ones that land inside the 1280×720 image.
4. Samples the pixel color under each kept point and transforms the points into `home`.
5. Bins the points into 0.25 m cells, padding the grid whenever a scan reaches past its edge.
6. Accumulates height and color sums and counts per cell with `index_add_`, then publishes the cell means as an XYZRGB `PointCloud2` on `/pcl`.

## Code

| Path | Contents |
|---|---|
| [`src/py_perception/py_perception/map_gen.py`](src/py_perception/py_perception/map_gen.py) | Map generator node |
| [`src/radar_drone_launch/launch/apm_altos.launch.py`](src/radar_drone_launch/launch/apm_altos.launch.py) | In-flight bring-up: Ouster, Orbbec, MAVROS, static TFs, `ros2 bag record` |
| [`src/radar_drone_launch/launch/mapgen.launch.py`](src/radar_drone_launch/launch/mapgen.launch.py) | Post-flight: static TF tree, bag replay, `map_gen`, RViz |
| [`src/radar_drone_launch/launch/mavros/`](src/radar_drone_launch/launch/mavros/) | MAVROS launch files, plugin allow-list and TF settings for ArduPilot |
| [`src/radar_drone_launch/config/`](src/radar_drone_launch/config/) | Ouster driver parameters (PTP timestamps, 1024x20 mode, 180° azimuth window), `ptp4l.conf`, RViz config |
| [`tools/clean_bag.py`](tools/clean_bag.py) | Removes `map → base_link` poses beyond 10 km and the recorded `/tf_static` before replay |
| [`Dockerfile`](Dockerfile) | ROS 2 Humble + PyTorch image that builds the workspace |

## Build and run

```bash
docker build -t skyplexus .
docker run --rm -it --gpus all skyplexus bash
source install/setup.bash
```

`map_gen` runs on CUDA, so it needs an NVIDIA GPU.

On the drone, `apm_altos.launch.py` also needs [MAVROS](https://github.com/mavlink/mavros), [ouster-ros](https://github.com/ouster-lidar/ouster-ros) and [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2):

```bash
ros2 launch radar_drone_launch apm_altos.launch.py
```

After the flight, clean the recorded bag (the input and output bag names are set in the script), then replay it through the mapper:

```bash
python3 tools/clean_bag.py
BAG=<cleaned bag> OFFSET=<start offset in seconds> ros2 launch radar_drone_launch mapgen.launch.py
```

## License

[MIT](LICENSE)
