Localization nodes {#localization-nodes}
=============

# Purpose / Use cases

A node with boilerplate to operate a localizer and manage the input and output data flows is needed.

# Design

[RelativeLocalizerNode](@ref autoware::localization::localization_nodes::RelativeLocalizerNode) is a generic relative
localization node template that manages a localizer implementation and a map implementation 
which respectively conforms to the interfaces
specified in [LocalizerConstraint](@ref 
autoware::localization::localization_nodes::traits::LocalizerConstraint) and [MapConstraint](@ref 
autoware::localization::localization_nodes::traits::MapConstraint)
and a map implementation that conforms to the inter, using a [PoseInitializerBase](@ref 
autoware::localization::localization_common::PoseInitializerBase) implementation. 

The type constraints are implemented as structs encapsulating series of static assertions which 
make the compilation fail with informant messages if the given algorithm implementation does 
not provide the required interfaces to the `RelativeLocalizerNode`. Please see the documentation 
of these constraint structs to understand the interface requirements for new 
implementations of localization and mapping algorithms.

* At each received observation message, the received message is registered in the localizer with the help of the fetched initial estimate and published.
* At each received map message, the map in the localizer is updated.



## Assumptions / Known limits

Since there are multiple callbacks, the node should be run in a single thread at any stage.

## Inputs / Outputs / API

Input:

- Map message
- Observation message
- Transform messages for initial estimate

Output:

- Output pose message


## Error detection and handling



## Security considerations


# Future extensions / Unimplemented parts


# Related issues

- #143 - Implement RelativeLocalizerBaseNode

