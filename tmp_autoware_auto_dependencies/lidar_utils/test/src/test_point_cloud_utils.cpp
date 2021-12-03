// Copyright 2017-2021 the Autoware Foundation, Arm Limited
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

#include <common/types.hpp>
#include <lidar_utils/lidar_utils.hpp>
#include <lidar_utils/point_cloud_utils.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

TEST(TestPointCloudUtils, HasIntensityAndThrowIfNoXyzTest)
{
  using autoware::common::lidar_utils::create_custom_pcl;
  using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;

  const uint32_t mini_cloud_size = 10U;

  std::vector<std::string> right_field_names{"x", "y", "z", "intensity"};
  std::vector<std::string> not_intensity_field_names{"x", "y", "z", "not_intensity"};
  std::vector<std::string> three_field_names{"x", "y", "z"};
  std::vector<std::string> five_field_names{"x", "y", "z", "intensity", "timestamp"};
  std::vector<std::string> invalid_field_names{"x", "y"};
  std::vector<std::string> wrong_x_field_names{"h", "y", "z"};
  std::vector<std::string> wrong_y_field_names{"x", "h", "z"};
  std::vector<std::string> wrong_z_field_names{"x", "y", "h"};
  const auto correct_pc = create_custom_pcl<float32_t>(right_field_names, mini_cloud_size);
  const auto not_intensity_pc = create_custom_pcl<float32_t>(
    not_intensity_field_names,
    mini_cloud_size);
  const auto three_fields_pc = create_custom_pcl<float32_t>(three_field_names, mini_cloud_size);
  const auto five_fields_pc = create_custom_pcl<float32_t>(five_field_names, mini_cloud_size);
  const auto invalid_pc = create_custom_pcl<float32_t>(invalid_field_names, mini_cloud_size);
  const auto no_x_pc = create_custom_pcl<float32_t>(wrong_x_field_names, mini_cloud_size);
  const auto no_y_pc = create_custom_pcl<float32_t>(wrong_y_field_names, mini_cloud_size);
  const auto no_z_pc = create_custom_pcl<float32_t>(wrong_z_field_names, mini_cloud_size);

  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(invalid_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_x_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_y_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_z_pc), std::runtime_error);
  EXPECT_FALSE(has_intensity_and_throw_if_no_xyz(not_intensity_pc));
  EXPECT_FALSE(has_intensity_and_throw_if_no_xyz(three_fields_pc));
  EXPECT_TRUE(has_intensity_and_throw_if_no_xyz(correct_pc));
  EXPECT_TRUE(has_intensity_and_throw_if_no_xyz(five_fields_pc));
}

TEST(TestStaticTransformer, TransformPoint)
{
  Eigen::Quaternionf rotation;
  rotation = Eigen::AngleAxisf(M_PI_2f32, Eigen::Vector3f::UnitZ());

  geometry_msgs::msg::Transform tf;
  tf.translation.x = 1.0;
  tf.translation.y = 2.0;
  tf.translation.z = 3.0;
  // Rotation around z axis by 90 degrees.
  tf.rotation.x = static_cast<float64_t>(rotation.x());
  tf.rotation.y = static_cast<float64_t>(rotation.y());
  tf.rotation.z = static_cast<float64_t>(rotation.z());
  tf.rotation.w = static_cast<float64_t>(rotation.w());
  autoware::common::types::PointXYZF point{5.0F, 5.0F, 5.0F};
  autoware::common::lidar_utils::StaticTransformer transformer{tf};
  autoware::common::types::PointXYZF result_point{};
  transformer.transform(point, result_point);
  EXPECT_FLOAT_EQ(-4.0F, result_point.x);
  EXPECT_FLOAT_EQ(7.0F, result_point.y);
  EXPECT_FLOAT_EQ(8.0F, result_point.z);
}
