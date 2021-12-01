# outlier_filter

## Purpose

TODO

## Inner-workings / Algorithms

TODO

| Filter Name                     | Description                                                                     | Detail                                          |
| ------------------------------- | ------------------------------------------------------------------------------- | ----------------------------------------------- |
| radius search 2d outlier filter | subscribe multiple pointclouds and concatenate them into a pointcloud           | [link](docs/raidus-search-2d-outlier-filter.md) |
| voxel grid outlier filter       | remove points within a given box                                                | [link](docs/voxel-grid-outlier-filter.md)       |
| ring outlier filter             | compensate pointcloud distortion caused by ego vehicle's movement during 1 scan | [link](docs/ring-outlier-filter.md)             |
| occupancy grid outlier filter   | downsampling input pointcloud                                                   | [link](docs/occupancy-grid-outlier-filter.md)   |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

[1] <https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html>

## (Optional) Future extensions / Unimplemented parts
