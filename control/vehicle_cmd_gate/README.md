# Vehicle Cmd Decider

## Purpose

`vehicle_cmd_decider` is a module to .

## Inputs / Outputs

### Input

| Name                      | Type                                                       | Description                  |
| ------------------------- | ---------------------------------------------------------- | ---------------------------- |
| `~/input/emergency_state` | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `~/input/steering`        | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `~/input/emergency_state` | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `~/input/emergency_state` | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `~/input/emergency_state` | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `~/input/emergency_state` | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |


### Output

| Name                | Type                                           | Description                        |
| ------------------- | ---------------------------------------------- | ---------------------------------- |
| `~output/shift_cmd` | `autoware_auto_vehicle_msgs::msg::GearCommand` | Gear for drive forward / backward. |

## Parameters

none.

## Assumptions / Known limits

TBD.
