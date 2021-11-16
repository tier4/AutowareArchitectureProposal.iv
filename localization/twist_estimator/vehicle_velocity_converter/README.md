# vehicle_velocity_converter

## Purpose

## Inputs / Outputs

### Input

| Name              | Type                                             | Description      |
| ----------------- | ------------------------------------------------ | ---------------- |
| `velocity_status` | `autoware_auto_vehicle_msgs::msg::VehicleReport` | vehicle velocity |

### Output

| Name                    | Type                                             | Description                                        |
| ----------------------- | ------------------------------------------------ | -------------------------------------------------- |
| `twist`                 | `geometry_msg::msg::TwistStamped`                | twist converted from VehicleReport message         |
| `twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | twist with covariance converted from VehicleReport |

## Parameters

| Name         | Type   | Description                               |
| ------------ | ------ | ----------------------------------------- |
| `frame_id`   | string | frame id for output message               |
| `covariance` | double | set covariance value to the twist message |
