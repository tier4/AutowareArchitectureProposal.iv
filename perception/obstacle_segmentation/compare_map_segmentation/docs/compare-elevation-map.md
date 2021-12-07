# compare_map_segmentation

## Purpose

## Inner-workings / Algorithms

<p align="center">
  <img src="./media/compare_elevation_map.png" width="1000">
</p>

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name                 | Type   | Description                                                                     | Default value |
| :------------------- | :----- | :------------------------------------------------------------------------------ | :------------ |
| `map_layer_name`     | string | elevation map layer name                                                        | elevation     |
| `map_frame`          | float  | frame_id of the map that is temporarily used before elevation_map is subscribed | map           |
| `height_diff_thresh` | float  | Remove points whose height difference is below this value [m]                   | 0.15          |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
