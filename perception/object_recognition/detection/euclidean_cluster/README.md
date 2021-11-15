# euclidean_cluster

## Purpose

euclidean_cluster is a package for clustering points into smaller parts to reduce processing time.

This package has two clustering methods: `euclidean_cluster` and `voxel_grid_based_euclidean_cluster`.

## Inner-workings / Algorithms

### euclidean_cluster

`pcl::EuclideanClusterExtraction` is applied to points. See [official document](https://pcl.readthedocs.io/en/latest/cluster_extraction.html) for details.

### voxel_grid_based_euclidean_cluster

`pcl::EuclideanClusterExtraction` is applied to points downsampled by `pcl::VoxelGrid`.

## Inputs / Outputs

### Input

| Name    | Type                            | Description      |
| ------- | ------------------------------- | ---------------- |
| `input` | `sensor_msgs::msg::PointCloud2` | input pointcloud |

### Output

| Name             | Type                                                        | Description                                  |
| ---------------- | ----------------------------------------------------------- | -------------------------------------------- |
| `output`         | `autoware_perception_msgs::msg::DetectedObjectsWithFeature` | cluster pointcloud                           |
| `debug/clusters` | `sensor_msgs::msg::PointCloud2`                             | colored cluster pointcloud for visualization |

## Parameters

### Core Parameters

| Name                          | Type  | Description                                                                                  |
| ----------------------------- | ----- | -------------------------------------------------------------------------------------------- |
| `use_height`                  | bool  | use point.z for clustering                                                                   |
| `min_cluster_size`            | int   | the minimum number of points that a cluster needs to contain in order to be considered valid |
| `max_cluster_size`            | int   | the maximum number of points that a cluster needs to contain in order to be considered valid |
| `tolerance`                   | float | the spatial cluster tolerance as a measure in the L2 Euclidean space                         |
| `voxel_leaf_size`             | float | the voxel leaf size of x and y (only for `voxel_grid_based_euclidean_cluster`)               |
| `min_points_number_per_voxel` | int   | the minimum number of points for a voxel (only for `voxel_grid_based_euclidean_cluster`)     |

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:
  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## (Optional) References/External links

<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (Optional) Future extensions / Unimplemented parts

The `use_height` option of `voxel_grid_based_euclidean_cluster` isn't implemented yet.
