# Autoware Global Parameter Loader

This package is to get global parameters.

## Usage

Add launch file to below description to get global parameters in the node.

```xml
<!-- Global parameters -->
  <include file="$(find-pkg-share autoware_global_parameter_loader)/launch/global_params.launch.py">
    <arg name="vehicle_model" value="$(var vehicle_model)"/>
  </include>
```

## Assumptions / Known limits

Currently only vehicle_info is loaded by this launcher.
