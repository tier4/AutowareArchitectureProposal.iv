NDT Map Provider Node - Usage and Configuration {#ndt-map-provider-node}
=============

# Usage

## 1. Necessary Files

You will need a map file in `.pcd` format containing the 3D point cloud data of the map.

You will also need a separate map information file in `.yaml` format. This contains the location of the point cloud map origin in geocentric coordinates.

The map information file (YAML file) should have the following format:

```
# example_map.yaml
---
map_config:
    latitude: 37.380811523812845
    longitude: -121.90840595108715
    elevation: 16.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

where `latitude`, `longitude` and `elevation` are the WGS84 coordinates of the origin of the point cloud map, and roll, pitch and yaw describe the orientation of the map reference frame.


## 2. Create/edit a parameter file to reference the  map YAML file

An example parameter file for the `map_publisher` node can be found in the following location (inside the ade environment):

`/opt/AutowareAuto/share/ndt_nodes/share/param/map_publisher.param.yaml`

Alternatively if building from source code the example parameter file will be located here:

`~/AutowareAuto/install/ndt_nodes/share/ndt_nodes/param/map_publisher.param.yaml`

Copy the file to another location and edit (and uncomment) the `map_pcd_file` and the `map_yaml_file` parameters of the file `map_publisher.param.yaml` to refer to the point cloud map and the YAML map information files:

```
   map_pcd_file: "/path/to/map_data.pcd"
   map_yaml_file: "/path/to/map_info.yaml"
```

Be carefull to enter appropriate values for map parameters such as minimum and maximum points.

## 3. Run the map_publisher node

Inside the ade, source the workspace. If you are using precompiled version source from `/opt/AutowareAuto/setup.bash`. if you are developing and have built from source code, source the from `~/AutowareAuto/install/setup.bash`

Run the node with the paramter file as an argument:

`ros2 run ndt_nodes ndt_map_publisher_exe --ros-args --params-file path/to/map_publisher.param.yaml`

Alternatively use a launch file:
`ros2 launch ndt_nodes map_provider.launch.py`

which launches the map provider node with the default parameter values in `map_publisher.param.yaml`, and also launches a `voxel_grid_node` to sub-sample the point cloud for visualization.

## 4. Output

By default (as defined in the parameter file), the map_provider node will publish the following information
- the ndt map on the topic `ndt_map`
- the point cloud map visualization on the topic `viz_ndt_map`
- if a `voxel_grid_node` is used to sub-sample the point cloud as in the launch file, it will be published on the topic `viz_map_subsampled`

# Configuration of map provider node


The configurable parameters passed by the parameter file `map_publisher.param.yaml` of the node are:

| Parameter Name | Usage |
|---|---|
|map_pcd_file | file name of point cloud data file |
|map_yaml_file | file name of map information file |
|map_frame | frame for map point coordinates |
|map_config.capacity | max ndt map voxel capacity |
|map_config.min_point.x | min x bound for ndt map data |
|map_config.min_point.y | min y bound for ndt map data |
|map_config.min_point.z | min z bound for ndt map data |
|map_config.max_point.x | max x bound for ndt map data |
|map_config.max_point.y | max y bound for ndt map data |
|map_config.max_point.z | max z bound for ndt map data |
|map_config.voxel_size.x | voxel x size for ndt map data |
|map_config.voxel_size.y | voxel y size for ndt map data |
|map_config.voxel_size.z | voxel z size for ndt map data |
|viz_map | flag for whether to publish map for visualization |

A voxel_grid_node is used to down sample the point cloud for visualization. An example parameter file for the voxel grid is given at

`/opt/AutowareAuto/share/ndt_nodes/share/param/pcl_map_voxel_grid_downsample.param.yaml`:
