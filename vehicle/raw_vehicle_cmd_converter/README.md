# The `raw vehicle command converter` Package

## scenario_selector_node

`raw_vehicle_command_converter` node is a node that convert desired acceleration and velocity to mechanical input by using feed forward + feed back control(optional).

### Input topics

| Name                         | Type                                                     | Description                                                                                                        |
| ---------------------------- | -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `~/input/control_cmd`        | autoware_auto_control_msgs::msg::AckermannControlCommand | target `velocity/acceleration/steering_angle/steering_angle_velocity` is necessary to calculate actuation command. |
| `~/vehicle/status/steering"` | autoware_auto_vehicle_msgs::SteeringReport                    | current status of steering used for steering feed back control                                                     |
| `~/input/twist`              | navigation_msgs::Odometry                                | twist topic in odometry is used.                                                                                   |

### Output topics

| Name                     | Type                                         | Description                                             |
| ------------------------ | -------------------------------------------- | ------------------------------------------------------- |
| `~/output/actuation_cmd` | autoware_vehicle_msgs::msg::ActuationCommand | actuation command for vehicle to apply mechanical input |

### Parameters

| Parameter                  | Type   | Description                                                                     |
| -------------------------- | ------ | ------------------------------------------------------------------------------- |
| `update_rate`              | double | timer's update rate                                                             |
| `th_max_message_delay_sec` | double | threshold time of input messages' maximum delay                                 |
| `th_arrived_distance_m`    | double | threshold distance to check if vehicle has arrived at the trajectory's endpoint |
| `th_stopped_time_sec`      | double | threshold time to check if vehicle is stopped                                   |
| `th_stopped_velocity_mps`  | double | threshold velocity to check if vehicle is stopped                               |

### Limitation

current feed back implementation is only applied to steering control.
