# compare_map_segmentation

## Purpose

The `compare_map_segmentation` is a node that filters the ground points from the input pointcloud by using map info (e.g. pcd, elevation map).

## Inner-workings / Algorithms

Detail description of each ground segmentation algorithm is in the following links.

| Filter Name       | Description | Detail                            |
| ----------------- | ----------- | --------------------------------- |
| ray_ground_filter |             | [link](docs/ray-ground-filter.md) |

## Inputs / Outputs

### Input

| Name                    | Type                            | Description      |
| ----------------------- | ------------------------------- | ---------------- |
| `~/input/points`        | `sensor_msgs::msg::PointCloud2` | reference points |
| `~/input/elevation_map` | `grid_map::msg::GridMap`        | elevation map    |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Node Parameters

| Name               | Type   | Default Value | Description                           |
| ------------------ | ------ | ------------- | ------------------------------------- |
| `input_frame`      | string | " "           | input frame id                        |
| `output_frame`     | string | " "           | output frame id                       |
| `max_queue_size`   | int    | 5             | max queue size of input/output topics |
| `use_indices`      | bool   | false         | flag to use pointcloud indices        |
| `latched_indices`  | bool   | false         | flag to latch pointcloud indices      |
| `approximate_sync` | bool   | false         | flag to use approximate sync option   |

## Assumptions / Known limits

`pointcloud_preprocessor::Filter` is implemented based on pcl_perception [1] because of [this issue](https://github.com/ros-perception/perception_pcl/issues/9).

## References/External links

[1] <https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/src/pcl_ros/filters/filter.cpp>
