// Copyright 2021 the Autoware Foundation
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

#include <gtest/gtest.h>

#include <lidar_utils/cluster_utils/point_clusters_view.hpp>

namespace
{

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware_auto_perception_msgs::msg::PointXYZIF;
using autoware::common::lidar_utils::PointClustersView;

using autoware_auto_perception_msgs::msg::PointClusters;

PointXYZIF make_point(float32_t x, float32_t y, float32_t z) noexcept
{
  PointXYZIF point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

}  // namespace

TEST(TestClusterView, EmptyMessage)
{
  PointClusters msg;
  PointClustersView view{msg};
  ASSERT_EQ(view.size(), 0UL);
  ASSERT_TRUE(view.empty());
  for (const auto & cluster_view : view) {
    (void)cluster_view;  // Make it look used to the compiler to avoid the warning.
    FAIL() << "There should be no clusters to iterate over";
  }
}

TEST(TestClusterView, IterateOverMessage)
{
  PointClusters msg;
  // Cluster 1
  msg.points.push_back(make_point(1.0F, 2.0F, 3.0F));
  msg.points.push_back(make_point(4.0F, 5.0F, 6.0F));
  msg.cluster_boundary.push_back(2U);
  // Cluster 2
  msg.points.push_back(make_point(7.0F, 8.0F, 9.0F));
  msg.cluster_boundary.push_back(3U);
  // Cluster 3 added as empty
  msg.cluster_boundary.push_back(3U);
  // Cluster 4
  msg.points.push_back(make_point(10.0F, 11.0F, 12.0F));
  msg.points.push_back(make_point(13.0F, 14.0F, 15.0F));
  msg.points.push_back(make_point(16.0F, 17.0F, 18.0F));
  msg.cluster_boundary.push_back(6U);
  // All clusters are in

  PointClustersView msg_view{msg};
  ASSERT_EQ(msg_view.size(), 4UL);

  ASSERT_EQ(msg_view[0UL].size(), 2UL);
  ASSERT_FALSE(msg_view[0UL].empty());
  ASSERT_EQ(msg_view[1UL].size(), 1UL);
  ASSERT_FALSE(msg_view[1UL].empty());
  ASSERT_EQ(msg_view[2UL].size(), 0UL);
  ASSERT_TRUE(msg_view[2UL].empty());
  ASSERT_EQ(msg_view[3UL].size(), 3UL);
  ASSERT_FALSE(msg_view[3UL].empty());

  const auto cluster_idx = 3UL;
  const auto & cluster_view = msg_view[cluster_idx];
  for (auto i = 0UL; i < cluster_view.size(); ++i) {
    // Checking [] operator
    EXPECT_FLOAT_EQ(cluster_view[i].x, msg.points[cluster_idx + i].x);
    EXPECT_FLOAT_EQ(cluster_view[i].y, msg.points[cluster_idx + i].y);
    EXPECT_FLOAT_EQ(cluster_view[i].z, msg.points[cluster_idx + i].z);
  }
  auto i = 0UL;
  for (const auto & point : cluster_view) {
    // Checking iterators
    EXPECT_FLOAT_EQ(point.x, msg.points[cluster_idx + i].x);
    EXPECT_FLOAT_EQ(point.y, msg.points[cluster_idx + i].y);
    EXPECT_FLOAT_EQ(point.z, msg.points[cluster_idx + i].z);
    ++i;
  }
}
