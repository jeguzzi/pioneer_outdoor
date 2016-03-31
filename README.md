# pioneer_outdoor

## Replay bag files

Use clock from bag files:

```bash
rosparam set use_sim_time true
rosbag play --clock <file_name>
```

## Republish imu

The node *imu_republisher.py* republishes [sensor_msgs/Imu](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Imu.html) messages from the topic
`in_imu` to the topic `out_imu`, setting timestamp  covariances and frame_id ("imu").


| parameter     | description      | default |
| ---           | ---   | --- |
| angle_cov     | the yaw variance | 3e-2
| angular_speed_cov  | the angular speed (yaw) variance | 5e-4 |


## Precache the maps used by rviz_satellite

cd in the cache folder

```bash
cd YOUR_WORKSPACE/src/rviz_satellite/mapscache/HASH
```

and run
```bash
roslaunch pioneer_outdoor rviz_satellite_prefetcher.launch
```

HASH is set by rviz_satellite based on the url format.
For instance, google maps with url

```python
http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}
```

have

```bash
HASH = 11941265828621334924
```

### Node rviz_satellite_prefetcher.py

| parameter     | description      |
| ---           | ---   |
| directory     | where to save the images |
| bounding_box  | two (WGS84) geographic points that define the area to cover with tiles
| z             | the tile zoom |
|url            | the tile API |
