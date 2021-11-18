# dummy_diag_publisher

## Purpose

This package outputs a dummy diagnostic data for debugging and developing.

## Inputs / Outputs

### Outputs

| Name           | Type                                     | Description         |
| -------------- | ---------------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs::msgs::DiagnosticArray` | Diagnostics outputs |

## Parameters

### Node Parameters

| Name          | Type   | Default Value | Explanation                           |
| ------------- | ------ | ------------- | ------------------------------------- |
| `update_rate` | int    | `10`          | Timer callback period [Hz]            |
| `diag_name`   | string | `diag_name`   | Diag_name set by dummy diag publisher |
| `is_active`   | bool   | `true`        | Force update or not                   |

## Assumptions / Known limits

TBD.

## Usage

```sh
ros2 launch dummy_diag_publisher dummy_diag_publisher.launch.xml
```
