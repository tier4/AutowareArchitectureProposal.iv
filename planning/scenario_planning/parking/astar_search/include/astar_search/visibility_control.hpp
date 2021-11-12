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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.


#ifndef ASTAR_SEARCH__VISIBILITY_CONTROL_HPP_
#define ASTAR_SEARCH__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(ASTAR_SEARCH_BUILDING_DLL) || \
  defined(ASTAR_SEARCH_EXPORTS)
    #define ASTAR_SEARCH_PUBLIC __declspec(dllexport)
    #define ASTAR_SEARCH_LOCAL
  #else
// defined(ASTAR_SEARCH_BUILDING_DLL) || defined(ASTAR_SEARCH_EXPORTS)
    #define ASTAR_SEARCH_PUBLIC __declspec(dllimport)
    #define ASTAR_SEARCH_LOCAL
  #endif
// defined(ASTAR_SEARCH_BUILDING_DLL) || defined(ASTAR_SEARCH_EXPORTS)
#elif defined(__linux__)
  #define ASTAR_SEARCH_PUBLIC __attribute__((visibility("default")))
  #define ASTAR_SEARCH_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define ASTAR_SEARCH_PUBLIC __attribute__((visibility("default")))
  #define ASTAR_SEARCH_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // ASTAR_SEARCH__VISIBILITY_CONTROL_HPP_
