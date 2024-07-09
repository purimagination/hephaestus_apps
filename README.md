# hephaestus_apps

```sh
roslaunch hephaestus_apps move_to_point.launch
```

## Sampling data

```sh
rosbag record /joint_states /camera/color/image_raw /camera/depth/image_rect_raw /aruco_single/result /tf
```