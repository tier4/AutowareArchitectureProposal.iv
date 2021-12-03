voxel_grid_filter {#voxel-grid-filter-design}
=================


# Voxel grid filter

The voxel grid filter is a form of point cloud downsampling that maintains the
majority of the information stored in the point cloud.

There are two forms:

- The approximate voxel grid filter
- The centroid voxel grid filter

Both voxel grid filters operate in the following manner:

- For each point, the voxel index is computed
    - This is accomplished by converting each real-valued Cartesian coordinate into
integer valued coordinates
- The integer coordinates are then converted into a single index
- The point is stored in the appropriate data structure


## Approximate voxel grid filter

Rather than the centroids of all points within a voxel, the point returned is
the centroid of the voxel itself. While less accurate, this removes an
additional scan over the data. Concretely, this is done in `pcl` in the
following way:

- Each point in the point cloud is considered
- Each point is assigned to a voxel
- If the voxel has no points assigned to it, the centroid of the voxel is pushed
to the output point cloud, otherwise it is ignored
- For non-spatial terms (e.g. color, intensity), the outgoing point takes on the
values of whatever incoming point activated said voxel


## Centroid voxel grid filter

The centroid voxel grid filter requires an additional pass through the data to
compute the centroid of each point assigned to a voxel. This results in a more
accurate representation of the underlying surface. Concretely, this is done in
`pcl` by doing the following:

- As above, each point is stored in a vector with an additional voxel index
field
- The array is then sorted according to voxel index
- The array is then scanned through, with centroids for each contiguous set of
points being computed for each outgoing voxel

## Modifications

The following modifications are made:

- Active voxels are stored in a hashmap for faster average case access time
- As points are received and indexed, the voxel centroids are incrementally
updated

Similarly, for the approximate voxel grid filter, a hashmap (unordered map) is
used to store active voxels. However, centroids are not tracked, and the hashmap
is only used to track if the voxel is active or not.

## Architecture

The following architecture was used to maximize code re-use and improve performance.

A core `Voxel` class was provided, with common functionality. This class has no virtual
methods. This class is templated on a point type.

Child instances of the `Voxel` class are instantiated with some additional, common methods
which differentiate the behavior of individual voxels.

The `VoxelGrid` data structure itself is templated on a Voxel type.

Further down the line, runtime dispatching of algorithms can be achieved by using the non-templated
`AlgorithmBase`, and templated child class paradigm.


Finally, a note on some API and structural decisions:

- An `unordered_map` was used as the underlying datastructure to have improved average lookup time
and match the sparseness of the data
- An `std::forward_list` was used because the `splice` method allows us to reuse points
- A const reference was provided as the output to `new_voxels` in order to prevent the state of the
voxel grid from unexpectedly being changed in addition to MISRA considerations


## Performance characterization

### Time

Inserting into the voxel grid filter is inserting into a hashmap, which is
`O(n)` in an adversarial case.

For the centroid voxel grid filter, this results in an algorithm that is
`O(n*n + n)` in complexity, where the first term is for inserting points into
the voxel grid and underlying hashmap, as well as updating the centroids,
and the second term is for flushing points out of the voxel grid.

This results in an algorithm that is `O(n^2)`

Similarly for the approximate voxel grid filter, the algorithm is `O(n * n)`
for inserting into the voxel grid, and sending newly activated points downstream.

This results in an algorithm that is `O(n^2)`.

In practice, inserting into a hashmap is `O(1)`, which results in `O(n)`
algorithms for both cases.

### Space

The space complexity of each voxel grid is dominated by the underlying hashmap,
which is `O(n)` in space.


# States

The voxel grid filters are stateful in two ways: the configuration, and the
voxel grid itself.


## Voxel state

- `Hashmap<uint32_t, Voxel>`, main data structure for storing a sparse voxel
grid, similar to an `std::unordered_map`
- `Voxel`, represents an exact voxel by maintaining two state variables:
`centroid` and `num_points`, to allow incremental updates of the centroid


## Configuration state

The following defines the configuration state:
- `min_point`
- `max_point`, combined with the above define an axis-aligned bounding box
(AABB) that all points will fall in
- `leaf_size`, defines the length of each side of the voxel

Further configuration state variables are precomputed from these for
computational convenience:
- `inv_leaf_size`, makes it quicker to compute voxel index by switching a
division for a multiply
- `leaf_size_2`, makes it quick to compute approximate voxel centroid
- `x_width`, number of voxels in the x direction, for quick computation of voxel
index and approximate centroid
- `xy_width`, number of voxels in each z row, for quick computation of voxel
index and approximate centroid


# Inputs

The input to the voxel grid data structure is an arbitrary point type that the voxel is
templated on.

This voxel must have the following properties for successful compilation:
- public float32_t members x, y, and z
- If using `CentroidVoxel`, `operator+` and `operator*(float32_t)` must be defined

# Outputs

The voxel grid data structure supports to output modes:

- Iteration over the entire voxel grid
- Getting newly active voxels from an output queue

# Security considerations

TBD by a security expert.

# Future Work

The underlying `std::unordered_map` is not static memory, and cannot be made so with default STL
capabilities. When available, a static memory allocator must be used.
