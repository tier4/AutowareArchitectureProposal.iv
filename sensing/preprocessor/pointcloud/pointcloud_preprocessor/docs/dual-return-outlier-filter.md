# radius_search_2d_outlier_filter

## Purpose

TODO

## Inner-workings / Algorithms

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name            | Type   | Description                                                                                                              |
| --------------- | ------ | ------------------------------------------------------------------------------------------------------------------------ |
| `min_neighbors` | int    | If points in the circle centered on reference point is less than `min_neighbors`, a reference point is judged as outlier |
| `search_radius` | double | Searching number of points included in `search_radius`                                                                   |

## Assumptions / Known limits

Since the method is to count the number of points contained in the cylinder with the direction of gravity as the direction of the cylinder axis, it is a prerequisite that the ground has been removed.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

[1]: https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html
[2]: https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search

## (Optional) Future extensions / Unimplemented parts
