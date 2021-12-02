# voxel_grid_outlier_filter

## Purpose

TODO

## Inner-workings / Algorithms

TODO

![voxel_grid_outlier_filter_picture](./image/outlier_filter-voxel_grid.drawio.svg)

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name                     | Type   | Default Value | Description                                |
| ------------------------ | ------ | ------------- | ------------------------------------------ |
| `voxel_size_x`           | double | 0.3           | the voxel size along x-axis [m]            |
| `voxel_size_y`           | double | 0.3           | the voxel size along y-axis [m]            |
| `voxel_size_z`           | double | 0.1           | the voxel size along z-axis [m]            |
| `voxel_points_threshold` | int    | 2             | the minimum number of points in each voxel |

## Assumptions / Known limits

TODO

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
