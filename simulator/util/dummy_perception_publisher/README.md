# dummy_perception_publisher

## Purpose

This node publishes a distance from the closest path point from the self-position to the end point of the path.
Note that the distance means the arc-length along the path, not the Euclidean distance between the two points.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                     | Type                                       | Description    |
| ------------------------ | ------------------------------------------ | -------------- |
| `/tf`                    | `tf2_msgs/TFMessage`                       | TF (self-pose) |
| `input/object`           | `dummy_perception_publisher::msg::Object`  | TF (self-pose) |

### Output

| Name         | Type                                       | Description                                                                                           |
| ------------ | ------------------------------------------ | ----------------------------------------------------------------------------------------------------- |
| `output/dynamic_object` | `autoware_perception_msgs::msg::DetectedObjectsWithFeature / autoware_auto_perception_msgs::msg::DetectedObjects` | Publish a distance from the closest path point from the self-position to the end point of the path[m] |
| `output/points_raw` | `sensor_msgs::msg::PointCloud2` | Reference path |

## Parameters
| Name          | Type   | Default Value | Explanation                 |
| ------------- | ------ | ------------- | --------------------------- |
| `visible_range` | double | 100.0          | sensor visible range [m] |
| `detection_successful_rate` | double | 0.8          | sensor visible range [m] |
| `enable_ray_tracing` | bool | true          | sensor visible range [m] |
| `use_object_recognition` | bool | true          | sensor visible range [m] |
| `real_use_param_` | bool | true          | sensor visible range [m] |

### Node Parameters
None.

### Core Parameters

None.

## Assumptions / Known limits

TBD.
