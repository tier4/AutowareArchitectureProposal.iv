# autoware_state_monitor

## Purpose

This node manages AutowareState transitions.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

  <arg name="input_autoware_engage" default="/api/autoware/get/engage" />
  <arg name="input_vehicle_state_report" default="/vehicle/state_report" />
  <arg name="input_hazard_status" default="/system/emergency/hazard_status" />
  <arg name="input_route" default="/planning/mission_planning/route" />

  <!-- Output -->
  <arg name="output_autoware_state" default="/autoware/state" />
  <arg name="output_autoware_engage" default="/autoware/engage" />

| Name                              | Type                                                  | Description                                       |
| --------------------------------- | ----------------------------------------------------- | ------------------------------------------------- |
| `/system/emergency/hazard_status` | `autoware_auto_system_msgs::msg::HazardStatusStamped` | Used to check autoware is emergency state or not  |
| `/planning/mission_planning/rout` | `autoware_auto_planning_msgs::msg::Route`             | Subscribe route                                   |
| `/localization/ekf_odom`          | `nav_msgs::msg::Odometry`                             | Used to decide whether vehicle is stopped or not  |
| `/vehicle/state_report`           | `autoware_auto_vehicle_msgs::msg::VehicleStateReport` | Used to check vehicle mode: autonomous or manual. |

### Output

| Name               | Type                                                    | Description                                        |
| ------------------ | ------------------------------------------------------- | -------------------------------------------------- |
| `/autoware/engage` | `autoware_auto_system_msgs::msg::EmergencyStateStamped` | publish AutowareState                              |
| `/autoware/state`  | `autoware_auto_system_msgs::msg::AutowareState`         | publish disengage flag on AutowareState transition |

## Parameters

### Node Parameters

| Name          | Type | Default Value | Explanation            |
| ------------- | ---- | ------------- | ---------------------- |
| `update_rate` | int  | `10`          | Timer callback period. |

### Core Parameters

| Name                      | Type   | Default Value | Explanation                                                                |
| ------------------------- | ------ | ------------- | -------------------------------------------------------------------------- |
| `th_arrived_distance_m`   | double | 1.0           | threshold distance to check if vehicle has arrived at the route's endpoint |
| `th_stopped_time_sec`     | double | 1.0           | threshold time to check if vehicle is stopped                              |
| `th_stopped_velocity_mps` | double | 0.01          | threshold velocity to check if vehicle is stopped                          |
| `disengage_on_route`      | bool   | true          | send diengage flag or not when the route is subscribed                     |
| `disengage_on_goal`       | bool   | true          | send diengage flag or not when the vehicle is arrived goal                 |

## Assumptions / Known limits

TBD.
