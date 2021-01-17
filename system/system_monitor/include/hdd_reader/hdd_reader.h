#ifndef HDD_READER_HDD_READER_H
#define HDD_READER_HDD_READER_H
/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file hdd_reader.h
 * @brief HDD reader definitions
 */

#include "boost/serialization/map.hpp"
#include "boost/serialization/serialization.hpp"
#include "boost/serialization/string.hpp"
#include <map>
#include <string>

/**
 * @brief HDD information
 */
struct HDDInfo
{
  int error_code_;      //!< @brief error code, 0 on success, otherwise error
  std::string model_;   //!< @brief Model number
  std::string serial_;  //!< @brief Serial number
  int temp_;            //!< @brief temperature(DegC)

  /**
   * @brief Load or save data members.
   * @param [inout] ar archive reference to load or save the serialized data members
   * @param [in] version version for the archive
   * @note NOLINT syntax is needed since this is an interface to serialization and
   * used inside boost serialization.
   */
  template<typename archive>
  void serialize(archive & ar, const unsigned /*version*/)  // NOLINT(runtime/references)
  {
    ar & error_code_;
    ar & model_;
    ar & serial_;
    ar & temp_;
  }
};

/**
 * @brief HDD information list
 */
typedef std::map<std::string, HDDInfo> HDDInfoList;

#endif  // HDD_READER_HDD_READER_H
