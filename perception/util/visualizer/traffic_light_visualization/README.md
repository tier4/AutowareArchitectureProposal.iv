# traffic_light_visualization 

## Purpose

<!-- Write the purpose of this package and briefly describe the features.

Example:
  {package_name} is a package for planning trajectories that can avoid obstacles.
  This feature consists of two steps: obstacle filtering and optimizing trajectory.
-->

## Inner-workings / Algorithms

## Inputs / Outputs

### traffic_light_map_visualizer

#### Input

| Name                 | Type                                                | Description          |
| -------------------- | --------------------------------------------------- | -------------------- |
| `~/input/tl_state` | `autoware_auto_perception_msgs::msg::TrafficSignalArray`           | status of traffic lights|
| `~/input/vector_map`  | `autoware_auto_mapping_msgs::msg::HADMapBin` | vector map            |

#### Output

| Name                  | Type                                      | Description         |
| --------------------- | ----------------------------------------- | ------------------- |
| `~/output/traffic_light` | `visualization_msgs::msg::MarkerArray` | marker array that indicates status of traffic lights |

### traffic_light_roi_visualizer

#### Input

| Name                 | Type                                                | Description          |
| -------------------- | --------------------------------------------------- | -------------------- |
| `~/input/image` | `sensor_msgs::msg::Image`           | input image |
| `~/input/rois`  | `autoware_auto_perception_msgs::msg::TrafficLightRoiArray` | input rois            |
| `~/input/rough/rois` (option)  | `autoware_auto_perception_msgs::msg::TrafficLightRoiArray` | input rois            |
| `~/input/tl_state`  | `autoware_auto_perception_msgs::msg::TrafficSignalArray` | status of traffic lights            |

#### Output

| Name                  | Type                                      | Description         |
| --------------------- | ----------------------------------------- | ------------------- |
| `~/output/image` | `sensor_msgs::msg::Image`           | output image with rois |

## Parameters

### traffic_light_map_visualizer

None

### traffic_light_roi_visualizer

#### Node Parameters

| Name                   | Type | Default Value | Description                     |
| ---------------------- | ---- | --|----------------------------- |
| `enable_fine_detection` | bool | false| whether to use fine detection |

See detail algorithm about the fine detection in [1].

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

[1] M. Sandler, A. Howard, M. Zhu, A. Zhmoginov and L. Chen, "MobileNetV2: Inverted Residuals and Linear Bottlenecks," 2018 IEEE/CVF Conference on Computer Vision and Pattern Recognition, Salt Lake City, UT, 2018, pp. 4510-4520, doi: 10.1109/CVPR.2018.00474.

## (Optional) Future extensions / Unimplemented parts
