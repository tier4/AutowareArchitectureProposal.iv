NDT Failure Analysis {#ndt-failure-analysis}
===========================================

@tableofcontents

# Introduction

The Normal Distributions Transform (NDT) is a localization algorithm, specifically a relative
localization algorithm (which localizes an observation with respect to a ground-truth map).

Autonomous driving is a safety-critical use case, one in which failures should be avoided or
mitigated to the largest extent possible. As localization is an integral part of the autonomous
driving stack, it is important that localization components are as close to fail-free as possible.

In order to satisfy this requirement, it is necessary to conduct a failure analysis of this
localization component to identify and generate mitigation procedures for failure modalities.

This document is broken into three sections:
1. General failure modes for the super-class of components which NDT falls into
2. Specific failure modes for the NDT algorithm
3. A summary of recommended mitigation actions

# Relative localization failure modes

The inputs to a relative localizer are as follows:
1. Reference map (NDT)
2. Transform tree w/history
3. Sensor input (point cloud)

Failures associated with each of the inputs are broadly explored below.

## Reference map

A reference map is the ground truth for which a relative localizer computes a transform with respect
to.

1. No reference map is available
    1. **Mitigation**: The component does not reach the ACTIVE or RUNNING state without a valid map
2. Reference map is wrong
    1. **Mitigation**: Validate map format before acceptance
    2. **Mitigation**: Security features to ensure integrity of input should be enabled
3. Reference map is no longer valid (i.e. old map, vehicle out of range of map)
    1. **Mitigation**: Other localization modes should be available to continue minimal safe
    operation
    2. **Mitigation**: The component should signal to the larger system before it reaches the
    end of a map
    3. **Mitigation**: Output should be validated to ensure localization errors from out-of-date
    map are identified

## Transform tree

A transform tree provides a cohesive source for an initial estimate of the ego position with
respect to the map frame.

1. Transform tree does not cover timestamp of current observation
    1. **Mitigation**: An implementation-defined strategy should be used, e.g. extrapolation,
    state estimation, using the last known position
2. Transform tree is wrong
    1. **Mitigation**: Security features to ensure integrity of system and inputs should be used
    2. **Mitigation**: If smoothness or continuity is a valid assumption on the target frames
    (depends on use case), then new transforms can be checked to ensure the proposed smoothness
    property is maintained (i.e. using a state estimator and determining if new observations are
    outliers)

## Sensor input

The sensor input is the instantaneous observation that is matched against the reference map.

1. No sensor input is available
    1. **Mitigation**: N/A (this is a primary input; no computation occurs without this data)
2. Sensor input is wrong (e.g. old, malformed, spoofed)
    1. **Mitigation**: Basic point cloud validation can be done (i.e. expected format)
    2. **Mitigation**: If smoothness is assumed, then the time stamp of the data can be checked for
    smoothness
    3. **Mitigation**: Security features should be enabled to ensure the integrity of the input

# NDT-specific failure modes

The general workflow of NDT matching is as follows:

1. Get initial guess
2. Transform input point cloud given current guess
3. Compute gradient of current guess
4. Compute step size using More-Thuente line search (including additional gradient and objective
function evaluations), check for step size convergence
5. Compute hessian
6. Compute search direction
7. Apply update to current solution
8. Check for convergence or maximum iterations; else go to 2.

Some potential failure modes for each step are generally explored below.

## Get initial guess

The initial guess is obtained by looking up the transform in the transform tree at the specified
time stamp. If the transform is unavailable, the determination of the transform is implementation
defined.

1. Transform tree is wrong
    1. **Mitigation**: Ensure integrity of inputs/communication
    2. **Mitigation**: If smoothness is assumed, check new observations against history
2. Transform lookup is wrong
    1. **Mitigation**: This can be tested
3. Initial guess is wrong (other strategies)
    1. **Migitation**: Other strategies can be tested
    2. **Mitigation**: If multiple strategies are used, the initial guess can be compared via
    state estimation or fusion algorithms to detect outliers

## Transform point cloud

At each iteration, the sensor input must be transformed according to the current estimated
transform.

1. Transform procedure is wrong
    1. **Mitigation**: This can be tested

## Compute gradient

The gradient term in NDT is a sum of products. The sum is over each point, and the product is over
several terms ([1], equation 6.12):
1. Likelihood, an exponential of three terms:
    1. Normal distribution weight term ([1], equation 6.8)
    2. Position error
    3. Inverse covariance
2. Inverse covariance
3. Error derivative (sine/cosine term, [1], equation 6.14, 6.17)
4. Position error
5. Weight terms ([1], equation 6.8)

While exponentials can produce unbounded outputs, if the positive-semidefinite property of
covariance matrices hold, then the inner term of the exponential should always be non-positive,
implying that the exponential term is upper-bounded at 1.

In general, the failure modes are then:
1. Incorrect implementation
    1. **Mitigation**: Testing
2. Numerical errors due to floating point arithmetic becoming saturated
    1. **Mitigation**: Use double precision for numerical algorithms
    2. **Mitigation**: Control the number of input points via downsampling techniques
    3. **Mitigation**: Cap potentially unbounded terms (i.e. exponential, error terms)
3. Numerical errors due to covariance inversion
    1. **Mitigation**: Check the condition number of matrices before inversion, apply regularization
    if needed
    2. **Mitigation**: Use stable/robust inversion algorithms

## Compute step size

The More-Thuente line search algorithm relies on success evaluations of the gradient and the
objective function of the NDT algorithm.

The objective function is a sum of probability terms, each of which is composed out of a subset
of the terms used to compute the gradient.

As such the primary failure mode is:

1. The line search algorithm is wrong
    1. **Mitigation**: The algorithm should be properly tested to ensure correctness

There may be additional failure modes in the line search algorithm, which should properly be
analyzed.

The remaining failure concerns are the same as in computing the gradient.

## Compute hessian

The hessian is similarly a sum of products. Its product is composed of more terms than the
gradient, but with the same component parts, with the addition of a second derivative of the
position error.

The computation of the second derivative has no unique failure modes.

The remaining failure concerns are the same as in computing the gradient.

## Compute search direction

The search direction is computed via a standard application of Newton's method:

\f[
  \Delta p \leftarrow H^{-1} g
\f]

The solution of this linear system is subject to the standard failure modes of numerical algorithms:

1. Ill-conditioned/infeasible problems
    1. **Mitigation**: Check condition number, regularize if necessary
    2. **Mitigation**: Use robust/stable algorithms for linear system solving
2. Numerical errors
    1. **Mitigation**: Use double precision

## Update solution

The solution update is a simple mathematical operation:

\f[
  p \leftarrow p + \alpha \Delta p
\f]

This update can be verified by testing for correctness.

## Convergence check

The loop exit check is three-fold:
1. Check if the objective function has a sufficiently small decrease
2. Check if the step size is sufficiently small
3. Check if the maximum number of iterations has been hit

The first two cases are standard optimization acceptance criterion, and are assumed to produce a
correct answer, provided the implementation is correct.

The last exit criterion, results in a transform with uncertain optimality guarantees. It is possible
to produce a large step with an associated large change in the objective value on the value
optimization iteration, producing an optimal value.

**Mitigation**: an additional check should be made on the output to ensure that the solution is
within the operating region of the algorithm. The NDT algorithm is understood to only reliably
produce solutions if the starting point is within a certain translation and rotation of the ground
truth ([1], pg. 92).

# Recommended Mitigations

Based on the failure analysis above, the following mitigation actions are suggested in the
implementation, deployment, and integration of the NDT algorithm:

Implementation:
1. Validate sensor/map input format
2. Determine if smoothness is a reasonable assumption:
    1. Validate sensor input time stamp
    2. Validate transform based on smoothness assumptions
3. Sufficiently test implementation
4. Implementation should match localization architecture; should output diagnostic warning when
near the edge of the current map
5. Implementation should have separate initialization and running states to protect invariants
6. Use double precision
7. Cap unbounded terms (i.e. exponential, error terms)
8. Check condition number of matrices before attempting to solve linear systems
9. Use robust solver algorithms for linear systems
10. Downsample or upper-bound input size
11. Check final update size to ensure it's within the algorithm operation region

Deployment/Integration:
1. Enable security features
2. Downsample or upper-bound input size
3. Test/verify components that produce inputs to this module
4. System should appropriately react to diagnostic warnings when the vehicle is near the edge of
the current map

# References

- [1] [The Three-Dimensional Normal-Distributions Transform – an Efficient Representation for Registration, Surface Analysis, and Loop Detection](http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf)
- [2] [Line Search Algorithms with Guaranteed Sufficient Decrease, More and Thuente](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.454.389&rep=rep1&type=pdf)
