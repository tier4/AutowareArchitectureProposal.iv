ndt {#ndt}
=============

# Purpose / Use cases

This package contains structures and functionality to allow localization with 3D lidar scans using Normal distribution transform matching. See [this article](http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf)
for further details of the implementation.

# Design

![ndt architecture](images/ndt_uml.svg)


## Map

### Algorithm Design

NDTMap, similar to [voxel_grid](@ref autoware::perception::filters::voxel_grid), is based on an `std::unordered_map` and
allows fast lookup given a point.

 NDT voxel map has 2 main types: [DynamicNDTMap](@ref autoware::localization::ndt::DynamicNDTMap) and
 [StaticNDTMap](@ref autoware::localization::ndt::StaticNDTMap).

[DynamicNDTMap](@ref autoware::localization::ndt::DynamicNDTMap) transforms a raw point cloud message into
an NDT map representation by passing the points to their corresponding voxels. Each voxel's centroid and covariance gets
computed with respect to the points that fall inside it. Covariance and centroid computation is done online with respect to [Welford's algorithm](https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance) .

Once [DynamicNDTMap](@ref autoware::localization::ndt::DynamicNDTMap) transforms a dense point cloud into voxels,
each voxel's centroid and covariance can be serialized into a `PointCloud2` message where the resulting point cloud is
sparse and only an intermediate representation of a transformed map. This point cloud can then be converted back into
an NDT map representation via [StaticNDTMap](@ref autoware::localization::ndt::StaticNDTMap)
 where no centroid/covariance is computed but only the points
are inserted into their corresponding voxels for lookup. Input point cloud is validated by the 
`point_cloud_msg_wrapper::PointCloudMsgWrapper<>` where each points is represented as the 
[PointWithCovariances](@ref autoware::localization::ndt::PointWithCovariances) class.

### Inputs / Outputs / API
 Inputs:
 * Pointcloud

 Outputs:
 * Set of voxels given a point.

## Scan

An NDT scan is a data structure to represent a lidar scan. The implementations depend on the optimization problem.
All implementations must provide basic iterating abilities which is necessary for all ndt optimization problems.

### P2DNDTScan

#### Algorithm Design
[P2DNDTScan](@ref autoware::localization::ndt::P2DNDTScan) is a wrapper around a vector of points(`Eigen::Vector3d`).
The class allows iterating through the internal container by exposing the iterators of its vector.


## Optimization Problem

### P2D Optimization Problem

#### Algorithm Design

The algorithm is defined in detail in the paper "The Three-Dimensional Normal-Distributions Transform" [Magnusson 2009].
The actual implementation is based on the [PCL](https://github.com/PointCloudLibrary/pcl/tree/master/registration/include/pcl/registration)
and [autoware.ai](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/tree/master/ndt_cpu) implementations.

A [CachedExpression](@ref autoware::common::optimization::CachedExpression) is used to represent the optimization problem. As a result,
score, jacobian and hessian are given the option to be computed all computed together to make use of the synergy stemming from the shared terms within the computation.

#### Inputs / Outputs / API
Inputs:
 * Scan
 * Map
Outputs:
 * Score
 * Jacobian
 * Hessian

## NDT Localizer

[NDTLocalizerBase](@ref autoware::localization::ndt::NDTLocalizerBase) implements the interface 
enforced by [LocalizerConstraint](@ref 
autoware::localization::localization_nodes::traits::LocalizerConstraint).
It is templated on scan, map and optimization problem types, allowing to register measurements using varying ndt methods. An implementation of
[NDTLocalizerBase](@ref autoware::localization::ndt::NDTLocalizerBase) can choose to override the message and guess validations as well as covariance computation steps.

[P2DNDTLocalizer](@ref autoware::localization::ndt::P2DNDTLocalizer) is the [NDTLocalizerBase](@ref autoware::localization::ndt::NDTLocalizerBase) implementation for P2D NDT objective.

### Inputs / Outputs / API
Inputs:
 * Scan
 * Map
 * Initial estimate
 * Optimizer
Outputs:
 * Pose with covariance.

# Related issues
- #137: NDT Map format validation
- #138: Implement NDTMapRepresentation
- #136: Implement NDT Map Publisher
- #166: Implement P2D NDT scan
