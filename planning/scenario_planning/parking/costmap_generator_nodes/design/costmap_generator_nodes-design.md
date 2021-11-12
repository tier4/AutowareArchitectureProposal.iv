Costmap generator node {#costmap-generator-nodes}
========

This is the design document for the `costmap_generator_nodes` package.

# Purpose / Use cases

Costmap generator provides an action server for generating costmap representation of an environment.

# Design

At the start of the node the costmap is initialized with basic information such as frame id and position within that
frame, resolution, layers, and size. Some of these properties may change, for example, size and position may be changed
as a result of costmap trimming.

The process of costmap generation consists of the following steps:

1. Configure the costmap generator algorithm with needed parameters.
2. Action server is called. The goal of the action is a route consisting of a start pose and a goal pose.
3. Bounds of the costmap are calculated as an area necessary to plan the requested route.
4. Costmap generator calls HAD map server requesting HAD map of a drivable areas within the calculated bounds.
5. Costmap generator receives the HAD map.
6. Translation between costmap and vehicle position is obtained.
7. Transform between map and costmap frame is obtained.
8. Costmap generator is called with information mentioned in 5-7 scope.
9. After receiving costmap it is converted into a defined communication message.
10. Debug visualizations are published.
11. Action succeeds sending the costmap as a result.

## Inputs / Outputs / API

### Input

### Output topics

| Name                    | Type                                  | Description                                        |
| ----------------------- | ------------------------------------- | -------------------------------------------------- |
| `~debug/occupancy_grid` | nav_msgs::msg::OccupancyGrid          | costmap as OccupancyGrid, values are from 0 to 100 |
| `~debug/viz_had_map`    | visualization_msgs::msg::MarkerArray  | part of the lanelet that costmap is based on       |

### Action Server

| Name               | Type                                       | Description                                                 |
| ------------------ | ------------------------------------------ | ----------------------------------------------------------- |
| `generate_costmap` | autoware_auto_msgs::action::PlannerCostmap | action server for generation of costmap for the given route |

### Service client

| Name                       | Type                                   | Description                                                           |
| -------------------------- | -------------------------------------- | --------------------------------------------------------------------- |
| `~/client/HAD_Map_Service` | autoware_auto_msgs::srv::HADMapService | client for HAD map containing the area between the start and the goal |


## Configuration

| Name                  | Type   | Description                                                    |
| --------------------- | ------ | -------------------------------------------------------------- |
| `use_wayarea`         | bool   | whether using `wayarea` from `~/client/HAD_Map_Service` or not |
| `bound_costmap`       | bool   | whether trim output costmap or not                             |
| `costmap_frame`       | string | created costmap's coordinate                                   |
| `vehicle_frame`       | string | vehicle's coordinate                                           |
| `map_frame`           | string | map's coordinate                                               |
| `grid_min_value`      | double | minimum cost for gridmap                                       |
| `grid_max_value`      | double | maximum cost for gridmap                                       |
| `grid_resolution`     | double | resolution for gridmap                                         |
| `grid_length_x`       | int    | size of gridmap for x direction                                |
| `grid_length_y`       | int    | size of gridmap for y direction                                |
| `grid_position_x`     | int    | offset from coordinate in x direction                          |
| `grid_position_y`     | int    | offset from coordinate in y direction                          |
| `route_box_padding_m` | double | padding of bounding box for route that determines costmap size |


# Future extensions / Unimplemented parts
