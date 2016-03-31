# pioneer_outdoor

## Replay bag files

Use clock from bag files:

```bash
rosparam set use_sim_time true
rosbag play --clock <file_name>
```


## Precache the maps used by rviz_satellite

cd in the cache folder

```bash
cd YOUR_WORKSPACE/src/rviz_satellite/mapscache/HASH
```

and run
```bash
roslaunch pioneer_outdoor rviz_satellite_prefetcher.launch
```

The HASH is set by rviz_satellite based on the url format.
For instance, google maps with url

```
http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}
```

have

```bash
HASH=11941265828621334924
```

### Node rviz_satellite_prefetcher.py

| Parameter     |       |
| ---           | ---   |
| directory     | where to save the images |
| bounding_box  | two (WGS84) geographic points that define the area to cover with tiles
| z             | the tile zoom |
|url            | the tile API |
