voxel_grid_nodes {#voxel-grid-nodes-design}
================

This is the design document for the `voxel_grid_nodes` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package generally instantiates the objects and classes defined in the `voxel_grid` package for
use with node constructs and general interprocess communication.

In short, this package exists to connect the `voxel_grid` family of objects to the larger `ROS 2`
infrastructure.

More concretely, the functionality in this package is needed because it allows us to downsample
large point clouds into smaller point clouds, while still retaining a large amount of
representational power.


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

This package has 3 parts:

1. Base "algorithm" classes for interacting with an underlying voxel grid through the lens of the PointCloud2 messages
2. Instances of the base "algorithm" classes for different kinds of voxel grids, e.g. Approximate
or Centroid
3. A node wrapper around the `PointCloud2` algorithm class

The use of a base "algorithm" class is motivated by the fact that the underlying
[VoxelGrid](@ref autoware::perception::filters::voxel_grid::VoxelGrid) class is templated, and has
a template-dependent API (insofar as types are concerned). This means in order to get runtime
dispatching/polymorphism (to keep the code base lighter and usage simpler), we must wrap the data
structure in a polymorphic base class.

## Assumptions / Known limits
<!-- Required -->

The following limitations are present:

- The node allocates memory during runtime due to the following elements:
   - Underlying `VoxelGrid` data structure
   - Copying `PointCloud2` header (due to a string `frame_id`)
   - General pub/sub allocates memory

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The "algorithm" API is generally straightforward, with a single method for input and a single
method for outputs. See the following API docs for more details:

- [VoxelCloudBase](@ref autoware::perception::filters::voxel_grid_nodes::algorithm::VoxelCloudBase)

As a node, [`VoxelCloudNode`](@ref autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode)
has a standard set of public facing methods.

The inputs are a single PointCloud2 topic, and the outputs are another PointCloud2 topic.


## Error detection and handling
<!-- Required -->

Exceptions are generally thrown on errors.

The functionality in this package generally does not have error handling, as error handling
is delegated to the underlying data structure.


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

TBD by a security specialist

# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->

- Memory allocation must be cleaned up
- Integration tests should be added

# Related issues
<!-- Required -->
