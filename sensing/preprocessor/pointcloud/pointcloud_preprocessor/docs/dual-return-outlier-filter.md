# dual_return_outlier_filter

## Purpose

TODO

## Inner-workings / Algorithms

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Output

| Name                                           | Type                                       | Description |
| ---------------------------------------------- | ------------------------------------------ | ----------- |
| `/dual_return_outlier_filter/frequency_image`  | `sensor_msgs::msg::Image`                  |             |
| `/dual_return_outlier_filter/visibility`       | `autoware_debug_msgs::msg::Float32Stamped` |             |
| `/dual_return_outlier_filter/pointcloud_noise` | `sensor_msgs::msg::Pointcloud2`            |             |

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name            | Type   | Description                                                                                                              |
| --------------- | ------ | ------------------------------------------------------------------------------------------------------------------------ |
| `min_neighbors` | int    | If points in the circle centered on reference point is less than `min_neighbors`, a reference point is judged as outlier |
| `search_radius` | double | Searching number of points included in `search_radius`                                                                   |

## Assumptions / Known limits

TODO

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

## (Optional) Future extensions / Unimplemented parts
