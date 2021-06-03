Trajectory Follower Package {#trajectory-follower-package-design}
=============================================

# Purpose

Generate control commands to follow a given Trajectory.

# Design

This functionality is decomposed into three nodes.
- `LateralController`: generates lateral control messages.
- `LongitudinalController`: generates longitudinal control messages.
- @subpage latlon-muxer-design : combines the lateral and longitudinal control commands
into a single control command.

@image html images/trajectory_follower-diagram.png "Overview of the Trajectory Follower package"
