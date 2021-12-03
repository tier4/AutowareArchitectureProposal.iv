Localization common {#localization-common}
=============

# Pose Initializer

Relative localizers need a good initial estimate of the pose to converge to a good solution within a reasonable time.
For this reason, a pose initializer is required to make an estimate of the pose of the vehicle.

## Algorithm Design
Alternative localization modalities provide a reliable source for pose initialization. That is why
the initializer will try to lookup the transform at a given time from a transform tree when possible.

If the transform at a given time is not available within the given transform tree, the initializer will
attempt to extrapolate the transform at a given time using a strategy that is implementation defined.
Following implementations are currently available:

* [BestEfforInitializer](@ref autoware::localization::localization_common::BestEffortInitializer): Returns the latest available
transform when extrapolation is required.


# Related issues

- #142 Implement RelativeLocalizerIntializerBase, and a simple implementation
