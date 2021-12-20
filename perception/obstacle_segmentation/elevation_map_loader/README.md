# elevation_map_loader

## Purpose

This package provides elevation map for compare_map_segmentation.

## Inner-workings / Algorithms

Generate elevation_map from subscribed pointcloud_map and vector_map and publish it.
Save the generated elevation_map locally and load it from next time.

The elevation value of each cell is the average value of z of the points of the lowest cluster.  
Cells with No elevation value can be inpainted using the values of neighboring cells.

<p align="center">
  <img src="./media/elevation_map.png" width="1500">
</p>

## Inputs / Outputs

### Input

| Name                   | Type                                         | Description                                |
| ---------------------- | -------------------------------------------- | ------------------------------------------ |
| `input/pointcloud_map` | `sensor_msgs::msg::PointCloud2`              | The point cloud map                        |
| `input/vector_map`     | `autoware_auto_mapping_msgs::msg::HADMapBin` | (Optional) The binary data of lanelet2 map |

### Output

| Name                         | Type                            | Description                                                          |
| ---------------------------- | ------------------------------- | -------------------------------------------------------------------- |
| `output/elevation_map`       | `grid_map_msgs::msg::GridMap`   | The elevation map                                                    |
| `output/elevation_map_cloud` | `sensor_msgs::msg::PointCloud2` | (Optional) The point cloud generated from the value of elevation map |

## Parameters

### Node parameters

| Name                              | Type        | Description                                                                                                | Default value |
| :-------------------------------- | :---------- | :--------------------------------------------------------------------------------------------------------- | :------------ |
| map_layer_name                    | std::string | elevation_map layer name                                                                                   | elevation     |
| param_file_path                   | std::string | GridMap parameters config                                                                                  | path_default  |
| elevation_map_file_path           | std::string | elevation_map file (bag2)                                                                                  | path_default  |
| map_frame                         | std::string | map_frame when loading elevation_map file                                                                  | map           |
| use_inpaint                       | bool        | Whether to inpaint empty cells                                                                             | true          |
| inpaint_radius                    | float       | Radius of a circular neighborhood of each point inpainted that is considered by the algorithm [m]          | 0.3           |
| use_elevation_map_cloud_publisher | bool        | Whether to publish `output/elevation_map_cloud`                                                            | false         |
| use_lane_filter                   | bool        | Whether to filter elevation_map with vector_map                                                            | false         |
| lane_margin                       | float       | Value of how much to expand the range of vector_map [m]                                                    | 0.5           |
| lane_height_diff_thresh           | float       | Only point clouds in the height range of this value from vector_map are used to generate elevation_map [m] | 1.0           |
| lane_filter_voxel_size_x          | float       | Voxel size x for calculating point clouds in vector_map [m]                                                | 0.04          |
| lane_filter_voxel_size_y          | float       | Voxel size y for calculating point clouds in vector_map [m]                                                | 0.04          |
| lane_filter_voxel_size_z          | float       | Voxel size z for calculating point clouds in vector_map [m]                                                | 0.04          |

### GridMap parameters

The parameters are described on `config/elevation_map_parameters.yaml`.

#### General parameters

| Name                                           | Type | Description                                                                                                  | Default value |
| :--------------------------------------------- | :--- | :----------------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/num_processing_threads | int  | Number of threads for processing grid map cells. Filtering of the raw input point cloud is not parallelized. | 12            |

#### Grid map parameters

See: <https://github.com/ANYbotics/grid_map/tree/ros2/grid_map_pcl>

Resulting grid map parameters.

| Name                                                     | Type  | Description                                                                                                                                                            | Default value |
| :------------------------------------------------------- | :---- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/grid_map/min_num_points_per_cell | int   | Minimum number of points in the point cloud that have to fall within any of the grid map cells. Otherwise the cell elevation will be set to NaN.                       | 3             |
| pcl_grid_map_extraction/grid_map/resolution              | float | Resolution of the grid map. Width and length are computed automatically.                                                                                               | 0.3           |
| pcl_grid_map_extraction/grid_map/height_type             | int   | The parameter that determine the elevation of a cell `0: Smallest value among the average values of each cluster`, `1: Mean value of the cluster with the most points` | 1             |
| pcl_grid_map_extraction/grid_map/height_thresh           | float | Height range from the smallest cluster (Only for height_type 1)                                                                                                        | 1.0           |

### Point Cloud Pre-processing Parameters

#### Rigid body transform parameters

Rigid body transform that is applied to the point cloud before computing elevation.

| Name                                                | Type  | Description                                                                                                             | Default value |
| :-------------------------------------------------- | :---- | :---------------------------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/cloud_transform/translation | float | Translation (xyz) that is applied to the input point cloud before computing elevation.                                  | 0.0           |
| pcl_grid_map_extraction/cloud_transform/rotation    | float | Rotation (intrinsic rotation, convention X-Y'-Z'') that is applied to the input point cloud before computing elevation. | 0.0           |

#### Cluster extraction parameters

Cluster extraction is based on pcl algorithms. See <https://pointclouds.org/documentation/tutorials/cluster_extraction.html> for more details.

| Name                                                         | Type  | Description                                                                            | Default value |
| :----------------------------------------------------------- | :---- | :------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/cluster_extraction/cluster_tolerance | float | Distance between points below which they will still be considered part of one cluster. | 0.2           |
| pcl_grid_map_extraction/cluster_extraction/min_num_points    | int   | Min number of points that a cluster needs to have (otherwise it will be discarded).    | 3             |
| pcl_grid_map_extraction/cluster_extraction/max_num_points    | int   | Max number of points that a cluster can have (otherwise it will be discarded).         | 1000000       |

#### Outlier removal parameters

See <https://pointclouds.org/documentation/tutorials/statistical_outlier.html> for more explanation on outlier removal.

| Name                                                       | Type  | Description                                                                    | Default value |
| :--------------------------------------------------------- | :---- | :----------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/outlier_removal/is_remove_outliers | float | Whether to perform statistical outlier removal.                                | false         |
| pcl_grid_map_extraction/outlier_removal/mean_K             | float | Number of neighbours to analyze for estimating statistics of a point.          | 10            |
| pcl_grid_map_extraction/outlier_removal/stddev_threshold   | float | Number of standard deviations under which points are considered to be inliers. | 1.0           |

#### Subsampling parameters

See <https://pointclouds.org/documentation/tutorials/voxel_grid.html> for more explanation on point cloud downsampling.

| Name                                                     | Type  | Description                             | Default value |
| :------------------------------------------------------- | :---- | :-------------------------------------- | :------------ |
| pcl_grid_map_extraction/downsampling/is_downsample_cloud | bool  | Whether to perform downsampling or not. | false         |
| pcl_grid_map_extraction/downsampling/voxel_size          | float | Voxel sizes (xyz) in meters.            | 0.02          |
