// Copyright 2021 The Autoware Foundation
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

#ifndef BEHAVIOR_VELOCITY_PLANNER__VISIBILITY_CONTROL_HPP_
#define BEHAVIOR_VELOCITY_PLANNER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(BEHAVIOR_VELOCITY_PLANNER_BUILDING_DLL) || defined(BEHAVIOR_VELOCITY_PLANNER_EXPORTS)
    #define BEHAVIOR_VELOCITY_PLANNER_PUBLIC __declspec(dllexport)
    #define BEHAVIOR_VELOCITY_PLANNER_LOCAL
  #else  // defined(BEHAVIOR_VELOCITY_PLANNER_BUILDING_DLL)
// || defined(BEHAVIOR_VELOCITY_PLANNER_EXPORTS)
    #define BEHAVIOR_VELOCITY_PLANNER_PUBLIC __declspec(dllimport)
    #define BEHAVIOR_VELOCITY_PLANNER_LOCAL
  #endif  // defined(BEHAVIOR_VELOCITY_PLANNER_BUILDING_DLL)
// || defined(BEHAVIOR_VELOCITY_PLANNER_EXPORTS)
#elif defined(__linux__)
  #define BEHAVIOR_VELOCITY_PLANNER_PUBLIC __attribute__((visibility("default")))
  #define BEHAVIOR_VELOCITY_PLANNER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define BEHAVIOR_VELOCITY_PLANNER_PUBLIC __attribute__((visibility("default")))
  #define BEHAVIOR_VELOCITY_PLANNER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // BEHAVIOR_VELOCITY_PLANNER__VISIBILITY_CONTROL_HPP_
