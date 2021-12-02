# outlier_filter

## Purpose

TODO

## Inner-workings / Algorithms

| Filter Name                     | Description                                                                     | Detail                                       |
| ------------------------------- | ------------------------------------------------------------------------------- | -------------------------------------------- |
| dual return outlier filter      |                                                                                 | [link](./dual-return-outlier-filter.md)      |
| occupancy grid outlier filter   | downsampling input pointcloud                                                   | [link](./occupancy-grid-outlier-filter.md)   |
| radius search 2d outlier filter | subscribe multiple pointclouds and concatenate them into a pointcloud           | [link](./radius-search-2d-outlier-filter.md) |
| ring outlier filter             | compensate pointcloud distortion caused by ego vehicle's movement during 1 scan | [link](./ring-outlier-filter.md)             |
| voxel grid outlier filter       | remove points within a given box                                                | [link](./voxel-grid-outlier-filter.md)       |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
