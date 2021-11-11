# ndt_pcl_modified

## Purpose

This package is a modified version of NDT library, which is implemented in Point Cloud Library (https://github.com/PointCloudLibrary/pcl).



## Change Points
  - You can get the Hessian matrix by getHessian(). 
  - You can get the estimated position for each iteration by getFinalTransformationArray().
  - It doesn't align the 6 axes at the same time, just processes the rotational component first, then processes the 6 axes. [experimental feature]