// Copyright 2017-2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef VOXEL_GRID__VISIBILITY_CONTROL_HPP_
#define VOXEL_GRID__VISIBILITY_CONTROL_HPP_


////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(VOXEL_GRID_BUILDING_DLL) || defined(VOXEL_GRID_EXPORTS)
    #define VOXEL_GRID_PUBLIC __declspec(dllexport)
    #define VOXEL_GRID_LOCAL
  #else  // defined(VOXEL_GRID_BUILDING_DLL) || defined(VOXEL_GRID_EXPORTS)
    #define VOXEL_GRID_PUBLIC __declspec(dllimport)
    #define VOXEL_GRID_LOCAL
  #endif  // defined(VOXEL_GRID_BUILDING_DLL) || defined(VOXEL_GRID_EXPORTS)
#elif defined(__linux__)
  #define VOXEL_GRID_PUBLIC __attribute__((visibility("default")))
  #define VOXEL_GRID_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define VOXEL_GRID_PUBLIC __attribute__((visibility("default")))
  #define VOXEL_GRID_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
  #error "Unsupported Build Configuration"
#endif  // defined(__WIN32)

#endif  // VOXEL_GRID__VISIBILITY_CONTROL_HPP_
