# RANSAC Ground Filter

## Purpose

This algorithm is SAC-based ground segmentation.

See: <https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html>

## Inner-workings / Algorithms

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

#### Core Parameters

| Name                    | Type   | Default Value | Description                                                     |
| ----------------------- | ------ | ------------- | --------------------------------------------------------------- |
| `base_frame`            | string | "base_link"   | base_link frame                                                 |
| `unit_axis`             | string | "z"           | The axis which we need to search ground plane                   |
| `max_iterations`        | int    | 1000          | The maximum number of iterations                                |
| `outlier_threshold`     | double | 0.01          | The distance threshold to the model [m]                         |
| `plane_slope_threshold` | double | 10.0          | The slope threshold to prevent mis-fitting [deg]                |
| `voxel_size_x`          | double | 0.04          | voxel size x [m]                                                |
| `voxel_size_y`          | double | 0.04          | voxel size y [m]                                                |
| `voxel_size_z`          | double | 0.04          | voxel size z [m]                                                |
| `height_threshold`      | double | 0.01          | The height threshold from ground plane for no ground points [m] |
| `debug`                 | bool   | false         | whether to output debug information                             |


## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
