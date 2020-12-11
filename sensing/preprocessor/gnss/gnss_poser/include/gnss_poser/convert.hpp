// Copyright 2020 Tier IV, Inc.
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
#ifndef GNSS_POSER__CONVERT_HPP_
#define GNSS_POSER__CONVERT_HPP_

#include <string>

#include "gnss_poser/gnss_stat.hpp"
#include "geo_pos_conv/geo_pos_conv.hpp"

#include "GeographicLib/Geoid.hpp"
#include "GeographicLib/MGRS.hpp"
#include "GeographicLib/UTMUPS.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "rclcpp/logging.hpp"

namespace GNSSPoser
{
enum class MGRSPrecision
{
  _10_KIRO_METER = 1,
  _1_KIRO_METER = 2,
  _100_METER = 3,
  _10_METER = 4,
  _1_METER = 5,
  _100_MIllI_METER = 6,
  _10_MIllI_METER = 7,
  _1_MIllI_METER = 8,
  _100MICRO_METER = 9,
};
// EllipsoidHeight:height above ellipsoid
// OrthometricHeight:height above geoid
double EllipsoidHeight2OrthometricHeight(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const rclcpp::Logger & logger)
{
  double OrthometricHeight;
  try {
    GeographicLib::Geoid egm2008("egm2008-1");
    OrthometricHeight = egm2008.ConvertHeight(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude,
      GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(
      logger, "Failed to convert Height from Ellipsoid to Orthometric" << err.what());
  }
  return OrthometricHeight;
}

GNSSStat NavSatFix2UTM(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const rclcpp::Logger & logger)
{
  GNSSStat utm;
  utm.coordinate_system = CoordinateSystem::UTM;

  try {
    GeographicLib::UTMUPS::Forward(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, utm.zone, utm.northup, utm.x, utm.y);

    utm.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_msg, logger);

    utm.latitude = nav_sat_fix_msg.latitude;
    utm.longitude = nav_sat_fix_msg.longitude;
    utm.altitude = nav_sat_fix_msg.altitude;
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(logger, "Failed to convert from LLH to UTM" << err.what());
  }
  return utm;
}

GNSSStat UTM2MGRS(
  const GNSSStat & utm, const MGRSPrecision & precision, const rclcpp::Logger & logger)
{
  constexpr int GZD_ID_size = 5;  // size of header like "53SPU"

  GNSSStat mgrs = utm;
  mgrs.coordinate_system = CoordinateSystem::MGRS;
  try {
    std::string mgrs_code;
    GeographicLib::MGRS::Forward(
      utm.zone, utm.northup, utm.x, utm.y, utm.latitude, static_cast<int>(precision), mgrs_code);
    mgrs.zone = std::stod(mgrs_code.substr(0, GZD_ID_size));
    mgrs.x = std::stod(mgrs_code.substr(GZD_ID_size, static_cast<int>(precision))) *
      std::pow(
      10, static_cast<int>(MGRSPrecision::_1_METER) -
      static_cast<int>(precision));                 // set unit as [m]
    mgrs.y = std::stod(
      mgrs_code.substr(
        GZD_ID_size + static_cast<int>(precision), static_cast<int>(precision))) *
      std::pow(
      10, static_cast<int>(MGRSPrecision::_1_METER) -
      static_cast<int>(precision));                 // set unit as [m]
    mgrs.z = utm.z;                                 // TODO(ryo.watanabe)
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(logger, "Failed to convert from UTM to MGRS" << err.what());
  }
  return mgrs;
}

GNSSStat NavSatFix2MGRS(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const MGRSPrecision & precision,
  const rclcpp::Logger & logger)
{
  const auto utm = NavSatFix2UTM(nav_sat_fix_msg, logger);
  const auto mgrs = UTM2MGRS(utm, precision, logger);
  return mgrs;
}

GNSSStat NavSatFix2PLANE(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const int & plane_zone,
  const rclcpp::Logger & logger)
{
  GNSSStat plane;
  plane.coordinate_system = CoordinateSystem::PLANE;
  geo_pos_conv geo;
  geo.set_plane(plane_zone);
  geo.llh_to_xyz(nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude);
  plane.x = geo.y();
  plane.y = geo.x();
  plane.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_msg, logger);
  return plane;
}
}  // namespace GNSSPoser

#endif  // GNSS_POSER__CONVERT_HPP_
