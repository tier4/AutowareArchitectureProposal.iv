// Copyright 2017-2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef LIDAR_UTILS__VISIBILITY_CONTROL_HPP_
#define LIDAR_UTILS__VISIBILITY_CONTROL_HPP_

#if defined(_MSC_VER) && defined(_WIN64)
  #if defined(LIDAR_UTILS_BUILDING_DLL) || defined(LIDAR_UTILS_EXPORTS)
    #define LIDAR_UTILS_PUBLIC __declspec(dllexport)
    #define LIDAR_UTILS_LOCAL
  #else  // defined(LIDAR_UTILS_BUILDING_DLL) || defined(LIDAR_UTILS_EXPORTS)
    #define LIDAR_UTILS_PUBLIC __declspec(dllimport)
    #define LIDAR_UTILS_LOCAL
  #endif  // defined(LIDAR_UTILS_BUILDING_DLL) || defined(LIDAR_UTILS_EXPORTS)
#elif defined(__GNUC__) && defined(__linux__)
  #define LIDAR_UTILS_PUBLIC __attribute__((visibility("default")))
  #define LIDAR_UTILS_LOCAL __attribute__((visibility("hidden")))
#elif defined(__GNUC__) && defined(__APPLE__)
  #define LIDAR_UTILS_PUBLIC __attribute__((visibility("default")))
  #define LIDAR_UTILS_LOCAL __attribute__((visibility("hidden")))
#else  // !(defined(__GNUC__) && defined(__APPLE__))
  #error "Unsupported Build Configuration"
#endif  // _MSC_VER

#endif  // LIDAR_UTILS__VISIBILITY_CONTROL_HPP_
