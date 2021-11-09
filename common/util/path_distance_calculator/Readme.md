# path_distance_calculator

## Purpose

This node publishes a distance from the path point closest to the self-position to the end point of the path.
Note that the distance means the arc-length along the path, not the Euclidean distance between the two points.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                                                              | Type                                     | Description    |
| ----------------------------------------------------------------- | ---------------------------------------- | -------------- |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | `autoware_auto_planning_msgs::msg::Path` | Reference path |
| `/tf`                                                             | `tf2_msgs/TFMessage`                     | TF (self-pose) |

### Output

| Name         | Type                                       | Description                                                                                         |
| ------------ | ------------------------------------------ | --------------------------------------------------------------------------------------------------- |
| `~/distance` | `autoware_debug_msgs::msg::Float64Stamped` | Publish a distance from the path point closest to the self-position to the end point of the path[m] |

## Parameters

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

TBD.
