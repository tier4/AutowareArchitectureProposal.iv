# as

`as` is the package to connect Autoware with Pacmod via pacmod_interface.

## pacmod interface

### Input / Output

#### Input topics

- From Autoware

  | Name                           | Type                                                     | Description                                           |
  | ------------------------------ | -------------------------------------------------------- | ----------------------------------------------------- |
  | `/control/control_cmd`         | autoware_auto_control_msgs::msg::AckermannControlCommand | lateral and longitudinal control command              |
  | `/control/gear_cmd`            | autoware_auto_vehicle_msgs::msg::GearCommand             | gear command                                          |
  | `/control/turn_indicators_cmd` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand   | turn indicators command                               |
  | `/control/hazard_lights_cmd`   | autoware_auto_vehicle_msgs::msg::HazardLightsCommand     | hazard lights command                                 |
  | `/vehicle/engage`              | autoware_auto_vehicle_msgs::msg::Engage                  | engage command                                        |
  | `/vehicle/actuation_cmd`       | autoware_vehicle_msgs::msg::ActuationCommandStamped      | actuation (accel/brake pedal, steering wheel) command |
  | `/control/emergency_cmd`       | autoware_vehicle_msgs::msg::VehicleEmergencyStamped      | emergency command                                     |

- From Pacmod

  | Name                                | Type                              | Description                                                             |
  | ----------------------------------- | --------------------------------- | ----------------------------------------------------------------------- |
  | `/pacmod/parsed_tx/steer_rpt`       | pacmod3_msgs::msg::SystemRptFloat | current steering wheel angle                                            |
  | `/pacmod/parsed_tx/wheel_speed_rpt` | pacmod3_msgs::msg::WheelSpeedRpt  | current wheel speed                                                     |
  | `/pacmod/parsed_tx/accel_rpt`       | pacmod3_msgs::msg::SystemRptFloat | current accel pedal                                                     |
  | `/pacmod/parsed_tx/brake_rpt`       | pacmod3_msgs::msg::SystemRptFloat | current brake pedal                                                     |
  | `/pacmod/parsed_tx/shift_rpt`       | pacmod3_msgs::msg::SystemRptInt   | current gear status                                                     |
  | `/pacmod/parsed_tx/turn_rpt`        | pacmod3_msgs::msg::SystemRptInt   | current turn indicators status                                          |
  | `/pacmod/parsed_tx/global_rpt`      | pacmod3_msgs::msg::GlobalRpt      | current status of other parameters (e.g. override_active, can_time_out) |

#### Output topics

- To Pacmod

  | Name                         | Type                              | Description                                           |
  | ---------------------------- | --------------------------------- | ----------------------------------------------------- |
  | `pacmod/as_rx/accel_cmd`     | pacmod3_msgs::msg::SystemCmdFloat | accel pedal command                                   |
  | `pacmod/as_rx/brake_cmd`     | pacmod3_msgs::msg::SystemCmdFloat | brake pedal command                                   |
  | `pacmod/as_rx/steer_cmd`     | pacmod3_msgs::msg::SystemCmdFloat | steering wheel angle and angular velocity command     |
  | `pacmod/as_rx/shift_cmd`     | pacmod3_msgs::msg::SystemCmdInt   | gear command                                          |
  | `pacmod/as_rx/raw_steer_cmd` | pacmod3_msgs::msg::SteerSystemCmd | raw steering wheel angle and angular velocity command |

- To Autoware

  | Name                               | Type                                                   | Description                                          |
  | ---------------------------------- | ------------------------------------------------------ | ---------------------------------------------------- |
  | `/vehicle/status/control_mode`     | autoware_auto_vehicle_mgs::msg::ControlModeReport      | control mode                                         |
  | `/vehicle/status/twist`            | autoware_auto_vehicle_mgs::msg::VelocityReport         | velocity                                             |
  | `/vehicle/status/steering`         | autoware_auto_vehicle_mgs::msg::SteeringReport         | steering wheel angle                                 |
  | `/vehicle/status/gear_cmd`         | autoware_auto_vehicle_mgs::msg::GearReport             | gear status                                          |
  | `/vehicle/status/turn_indicators`  | autoware_auto_vehicle_mgs::msg::TurnIndicatorsReport   | turn indicators status                               |
  | `/vehicle/status/hazard_lights`    | autoware_auto_vehicle_mgs::msg::HazardLightsReport     | hazard lights status                                 |
  | `/vehicle/status/actuation_status` | autoware_auto_vehicle_mgs::msg::ActuationStatusStamped | actuation (accel/brake pedal, steering wheel) status |

### ROS Parameters

| Name                              | Type   | Description                                                                               |
| --------------------------------- | ------ | ----------------------------------------------------------------------------------------- |
| `base_frame_id`                   | string | frame id (assigned in pacmod command, but it does not make sense)                         |
| `command_timeout_ms`              | double | timeout [ms]                                                                              |
| `loop_rate`                       | double | loop rate to publish commands                                                             |
| `steering_offset`                 | double | steering wheel angle offset                                                               |
| `enable_steering_rate_control`    | bool   | when enabled, max steering wheel rate is used for steering wheel angular velocity command |
| `emergency_brake`                 | double | brake pedal for emergency                                                                 |
| `vgr_coef_a`                      | double | coefficient to calculate steering wheel angle                                             |
| `vgr_coef_b`                      | double | coefficient to calculate steering wheel angle                                             |
| `vgr_coef_c`                      | double | coefficient to calculate steering wheel angle                                             |
| `accel_pedal_offset`              | double | accel pedal offset                                                                        |
| `brake_pedal_offset`              | double | brake pedal offset                                                                        |
| `max_throttle`                    | double | max accel pedal                                                                           |
| `max_brake`                       | double | max brake pedal                                                                           |
| `max_steering_wheel`              | double | max steering wheel angle                                                                  |
| `max_steering_wheel_rate`         | double | max steering wheel angular velocity                                                       |
| `min_steering_wheel_rate`         | double | min steering wheel angular velocity                                                       |
| `steering_wheel_rate_low_vel`     | double | min steering wheel angular velocity when velocity is low                                  |
| `steering_wheel_rate_low_stopped` | double | min steering wheel angular velocity when velocity is almost 0                             |
| `low_vel_thresh`                  | double | threshold velocity to decide the velocity is low for `steering_wheel_rate_low_vel`        |
| `hazard_thresh_time`              | double | threshold time to keep hazard lights                                                      |
