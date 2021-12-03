Lidar integration {#lidar-integration-design}
=============================

This is the design document for the `lidar_integration` package.


# Purpose / Use cases

This package contains testing utilities for 3D perception nodes.

The purpose is to facilitate various forms of testing of the nodes.


# Design

This package provides spoofers, listeners, and some utility functionality to tie it all together.

Spoofers include the lidar_integration::Vlp16IntegrationSpoofer,
which mimics the (UDP) wire protocol of a Velodyne VLP16-Hires sensor in a very regular, synthetic
pattern. In addition, there is the lidar_integration::PointCloudMutationSpoofer, which randomly
generates point clouds, for use in fuzz testing.

The lidar_integration::LidarIntegrationListener family of nodes provides test
node which check for a certain periodicity and size of data.

Finally, lidar_integration::lidar_integration_test provides a simple way to run several nodes
together so that they can be tested in a single-executable, or unit testing environment.
