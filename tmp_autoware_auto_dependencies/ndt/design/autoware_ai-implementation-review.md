Review Autoware.AI NDT implementation {#ndt-review}
===========================================

# NDT localization workflow

## Higher level workflow

1. [Set input point cloud](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L958)
   1. [Copy the pointer to the pointcloud message](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L97)
   1. [Build the voxel grid](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L101-102)
1. Calculate the initial guess:
   * No sensors:
      * [Vehicle kinematic state is derived at the end of each ndt update using the transform and the time deltas](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L1172-1213). Only yaw is derived.
      * [Initial guess is estimated using the velocities and acceleration depending on the motion model](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L971-991). (Either linear with velocity or quadratic with the acceleration added.)
   * IMU available:
     * [Use the IMU data to have a translation(quadratic motion model) and orientation update.](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L770-814)
     * Uses the derived velocity from the ndt update.
   * Vehicle odometry available:
     * [Use odometry data to do a position update](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L738-765)(constant velocity model + orientation update.)
   * IMU and Vehicle odometry available:
    * [Identical to imu-only update, only use the velocity info from vehicle odometry instead.](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L706-733)
1. [Execute NDT optimization loop starting with the initial guess](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L1045)
   * See low level workflow
1. [Set the vehicle pose state to the resulting base_link pose](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L1144-1152)
   * Calculate the translation difference between the initial guess and the ndt estimate:
     * If it's bigger than a threshold, use the initial guess.
     * Else, use the ndt estimate.
1. [Calculate the velocity and acceleration using this estimate](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L1163-1202)
   * Mostly first level derivation by dividing the transform delta to the time delta.
   * Acceleration is estimated using the difference between the current and the previous velocity estimates.
1. [Publish the estimates](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L1359-1364)
   * Publish all of the initial guesses(despite only one of them should be getting calculated).
   * Publish the ndt estimate.
     * Published on the base_link and
   *[ Publish a reliability score](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L1415-1417)
1. [Add the transform to the frame graph.](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L1370-1377)


Notes:
* A fixed transform is applied to the initial guess before ndt matching. The authors refer to the estimates with this transformation applied, the `localizer` estimates and when the estimate is transformed back to its original frame, it's called the `ndt` estimate.
  * This static transform is a global variable and is not read from the transform graph.

## Lower Level Workflow

Follows the equations from [this](http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf) paper, section 6.2.

1. [Compute the gauss parameters](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L112-118)
   * The gaussian probability function is modified to account for outliers. See Eq. 6.7 and 6.8 on details of this.
1. [Transform the scanned cloud using the initial guess](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L120-124).
1. [Compute the gradients and Hessian analytically](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L140). See page 63 on the [paper](http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf) for exact mathematical details.
   1. [Compute coefficients for computing the Jacobian and Hessian matrices](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L305-426). (Eq. 6.19)
   1. For each point in the translated point cloud:
     1. Search for a list of neighbors for a fixed radius in the voxel grid:
     1. For each neighboring voxel
        1. [Compute the first and second order derivatives using the pre-calculated coefficients](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L235-266)
           * [Using the point derivatives and pre-computed gauss paramters, update the elements of the Hessian matrix.](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L269-300)
           * [Update the gradient of the NDT score function](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L288). (Eq. 6.12)
1. [Using the Hessian and the probability score gradient, the pose update is calculated according to Newton's method.](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L147-149)
    * $`H\Delta p = -g`$ : Here the $`H`$ is the hessian matrix, $`\Delta p`$ is the pose delta and $`g`$ is the gradient of the NDT score function that was calculated with the rest of derivatives in the previous steps.
    * This linear equation is solved by applying [JacobiSVD](https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html) to the Hessian matrix and then solving for the negative score gradient $`-g`$.
1. [Normalize the pose delta](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L159)
1. [Calculate the step length](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L160)
   * Moré-Thuente line search is used for estimating this parameter.
   * [Derivatives are re-calculated during the step-size search at each iteration.](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L503)
   * Hessian is updated after a step-length is selected.
1. [Udate the pose](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L168)
1. [If the max number of iterations are reached or the step size is smaller than an epsilon, stop the loop.](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L172-174) Else go to step 4.
   * The paper suggests an epsilon of 1e-6 but the implementation uses 0.1


## Positive Impressions
* Concerns are somewhat [separated](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/tree/master/ndt_cpu/include/ndt_cpu) on the algorithm level.
* Having a base class of [registration](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/include/ndt_cpu/Registration.h) that the [NDT entity](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/include/ndt_cpu/NormalDistributionsTransform.h)  is derived from seems like the start of a good inheritance structure for future extensions.
* Parameter naming and calculations are taken from an actual [paper](http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf) making it easier to read, review and reproduce.


## Negative Impressions

High level workflow:

* Minimal documentation/comments.
* Relies heavily on [global variables](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L81-239) that get modified by different static callbacks.
* Almost entire high level workflow of ndt localization occurs in the [pointcloud callback](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L925) without utilizing any helper functions.
  * Low readability.
  * Hard to distinguish separate functionality.
  * Interleaved steps.
* A lot of repeating code for [different configurations/implementations](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp#L1025-1096).
* Somewhat cryptic parameter names.
* Vehicle kinematic state relies on differentiating two consecutive transforms over a time delta.

Low level workflow:
* Redundant operations occur in the run-time loop:
   * [ Parameter calculations](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/blob/master/ndt_cpu/src/NormalDistributionsTransform.cpp#L112-118) could occur only when they are updated rather than on every computation.
* Optimization loop is interleaved with NDT specific steps.
