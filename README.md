# Get Started
enable graphic user interface in docker: `xhost +`
## in docker
```
roscore
```
## calib
```
roslaunch livox_camera_calib calib.launch 
```
## slam
```
roslaunch r3live r3live_bag.launch
rosbag play data/basic2_and_3_multi_source_slam/test5
```
after finishing the slam, you can press "S" to save the map, and use the following command to reconstruct the map.
```
roslaunch r3live r3live_reconstruct_mesh.launch
```
## advance4
open the config file in `src/livox_camera_calib/config/calib.yaml`, and change the `common.from_bag` to true, and
```
roslaunch livox_camera_calib calib.launch 
```
## advance5&6
we don't have rosbag to calibrate the sensors in advance and our data, so just use rough extrinsics.
```
roslaunch r3live r3live_bag.launch
rosbag play data/basic2_and_3_multi_source_slam/test5
```

