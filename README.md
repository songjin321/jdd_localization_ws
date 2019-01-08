### JD Competition Lidar Localization Workspace

## Install
```
cd jd_localization
catkin build
. devel/setup.bash
```

## Run
```
roslaunch hdl_localization hdl_localization.launch
## wait about 10 s
roslaunch cartographer_ros my_robot_odom.launch bag_filename:=/home/neousys/Data/jdd/test2.bag
```

## Result

<img src="src/hdl_localization/data/lidar_localization2.png" height="256pix" /> 
<img src="src/hdl_localization/data/lidar_localization3.png" height="256pix" /> 

The algorithm takes about 400ms in our dual-core i5 NUC for once lidar localization.

## The description of each package

### hdl_localization
inputs: 
- point cloud map with transform map -> utm
- lidar data
- robot model static tf
- A small piece of GPS signal at the initial moment give gps->
lidar odometry

outputs: output -> map and result.csv with output -> utm

### The Processor package contains programs and scripts for data processing.
- output.py convert lidar->map to output_link -> utm
- extract_jd_part: extract a 30m square map from jd cloud map
- extract_jd_part: extract a 20m square map from jd cloud map  
- icp_example: use icp algorithm in pcl library to registration two submaps
- ndt_omp: use ndt algorithm to registration two submaps
- separate_ground: separate object and ground from a map
- show: show two map in same frame
- voxel_grid: downsampling the point cloud with the given precision