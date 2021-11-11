// Copyright 2019 Christopher Ho
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

#ifndef MOTION_COMMON__VISIBILITY_CONTROL_HPP_
#define MOTION_COMMON__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(MOTION_COMMON_BUILDING_DLL) || defined(MOTION_COMMON_EXPORTS)
    #define MOTION_COMMON_PUBLIC __declspec(dllexport)
    #define MOTION_COMMON_LOCAL
  #else  // defined(MOTION_COMMON_BUILDING_DLL) || defined(MOTION_COMMON_EXPORTS)
    #define MOTION_COMMON_PUBLIC __declspec(dllimport)
    #define MOTION_COMMON_LOCAL
  #endif  // defined(MOTION_COMMON_BUILDING_DLL) || defined(MOTION_COMMON_EXPORTS)
#elif defined(__linux__)
  #define MOTION_COMMON_PUBLIC __attribute__((visibility("default")))
  #define MOTION_COMMON_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MOTION_COMMON_PUBLIC __attribute__((visibility("default")))
  #define MOTION_COMMON_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // MOTION_COMMON__VISIBILITY_CONTROL_HPP_
