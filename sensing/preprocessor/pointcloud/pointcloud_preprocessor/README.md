# pointcloud_preprocessor

## Purpose

The `pointcloud_preprocessor` is a package that includes filters for denoising, cropping and concatenating pointclouds, and correcting distortions. Downsampling filter is also included in this package.

## Inner-workings / Algorithms

Detail description of each filter's algorithm is in the following links.

| Filter Name            | Description                                                                        | Detail                                 |
| ---------------------- | ---------------------------------------------------------------------------------- | -------------------------------------- |
| concatenate_data       | subscribe multiple pointclouds and concatenate them into a pointcloud              | [link](docs/concatenate-data.md)       |
| crop_box_filter        | remove points within a given box                                                   | [link](doc/crop-box-filter.md)         |
| distortion_corrector   | compensate pointcloud distortion caused by ego vehicle's movement during 1 scan    | [link](docs/distortion-corrector.md)   |
| downsample_filter      | downsampling input pointcloud                                                      | [link](docs/downsample-filter.md)      |
| outlier_filter         | remove points caused by hardware problems, rain drops and small insects as a noise | [link](docs/outlier-filter.md)         |
| passthrough_filter     | remove points on the outside of a range in given field (e.g. x, y, z, intensity)   | [link](docs/passthrough-filter.md)     |
| pointcloud_accumulator | accumulate pointclouds for a given amount of time                                  | [link](docs/pointcloud-accumulator.md) |
| vector_map_filter      | remove points on the outside of lane by using vector map                           | [link](docs/vector-map-filter.md)      |

## Inputs / Outputs

### Input

| Name              | Type                            | Description       |
| ----------------- | ------------------------------- | ----------------- |
| `~/input/points`  | `sensor_msgs::msg::PointCloud2` | reference points  |
| `~/input/indices` | `pcl_msgs::msg::Indices`        | reference indices |

### Output

| Name              | Type                                      | Description     |
| ----------------- | ----------------------------------------- | --------------- |
| `~/output/points` | `autoware_planning_msgs::msg::Trajectory` | filtered points |

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

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
