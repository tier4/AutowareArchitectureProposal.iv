autoware_testing {#autoware_testing-package-design}
===========

This is the design document for the `autoware_testing` package.

# Purpose / Use cases

The package aims to provide a unified way to add standard testing functionality to the package, currently supporting:
- Smoke testing (`add_smoke_test`): launch a node with default configuration and ensure that it starts up and does not crash.

# Design

Uses `ros_testing` (which is an extension of `launch_testing`) and provides some parametrized, reusable standard tests to run.

## Assumptions / Known limits

Parametrization is limited to package and executable names. Test namespace is set as 'test'.
Parameters file for the package is expected to be in `param/test.param.yaml`.

## Inputs / Outputs / API

To add a smoke test to your package tests, add test dependency on `autoware_testing` to `package.xml`

```{xml}
<test_depend>autoware_testing</test_depend>
```

and add the following two lines to `CMakeLists.txt` in the `IF (BUILD_TESTING)` section:

```{cmake}
find_package(autoware_testing REQUIRED)
add_smoke_test(${PROJECT_NAME} <EXECUTABLE_NAME>)
```

Where `<EXECUTABLE_NAME>` is to be replaced by a desired string or expression.

Example test output:

```
 1/10 Test  #1: smoke_test .......................   Passed    5.61 sec
```

# References / External links
- https://en.wikipedia.org/wiki/Smoke_testing_(software)
- https://github.com/ros2/ros_testing
- https://github.com/ros2/launch/blob/master/launch_testing

# Future extensions / Unimplemented parts

- extending parametrization to also include custom parameter file location (if needed)
- Adding more types of standard tests.

# Related issues
- Issue #700: add smoke test
