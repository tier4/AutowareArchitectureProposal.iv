// Copyright 2021 Apex.AI, Inc.
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

#include <benchmark/benchmark.h>
#include <helper_functions/float_comparisons.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>

namespace
{

constexpr auto kCloudSize = 100UL;

sensor_msgs::msg::PointCloud2 create_point_cloud_through_wrapper(const std::size_t size)
{
  sensor_msgs::msg::PointCloud2 msg;
  using autoware::common::types::PointXYZIF;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg, "frame_id"};
  modifier.resize(size);
  return msg;
}

sensor_msgs::msg::PointCloud2 create_point_cloud_through_utils(const std::size_t kCloudSize)
{
  sensor_msgs::msg::PointCloud2 msg;
  using autoware::common::types::PointXYZIF;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg, "frame_id"};
  modifier.reserve(kCloudSize);
  modifier.clear();
  return msg;
}

}  // namespace

static void BenchMsgWrapperAddPointToCloud(benchmark::State & state)
{
  using autoware::common::types::PointXYZIF;
  auto msg = create_point_cloud_through_wrapper(kCloudSize);
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg};
  const PointXYZIF point{};
  for (auto _ : state) {
    for (auto i = 0U; i < kCloudSize; ++i) {
      modifier[i] = point;
    }
    benchmark::DoNotOptimize(msg);
  }
}

static void BenchMsgWrapperResizeAndAddPointToCloud(benchmark::State & state)
{
  using autoware::common::types::PointXYZIF;
  auto msg = create_point_cloud_through_wrapper(kCloudSize);
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg};
  const PointXYZIF point{};
  for (auto _ : state) {
    modifier.clear();
    modifier.resize(kCloudSize);
    for (auto i = 0U; i < kCloudSize; ++i) {
      modifier[i] = point;
    }
    benchmark::DoNotOptimize(msg);
  }
}

static void BenchMsgWrapperPushBackPointToCloud(benchmark::State & state)
{
  using autoware::common::types::PointXYZIF;
  auto msg = create_point_cloud_through_wrapper(kCloudSize);
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg};
  const PointXYZIF point{};
  for (auto _ : state) {
    modifier.clear();
    modifier.reserve(kCloudSize);
    for (auto i = 0U; i < kCloudSize; ++i) {
      modifier.push_back(point);
    }
    benchmark::DoNotOptimize(msg);
  }
}


static void BenchMsgWrapperAccessPoint(benchmark::State & state)
{
  using autoware::common::types::PointXYZIF;
  auto msg = create_point_cloud_through_wrapper(kCloudSize);
  const PointXYZIF point{};
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg};
  modifier.resize(kCloudSize);
  for (auto & p : modifier) {
    p = point;
  }
  auto x = 0.0F;
  auto y = 0.0F;
  auto z = 0.0F;
  auto intensity = 0.0F;
  auto id = 0;
  for (auto _ : state) {
    for (const auto & p : modifier) {
      benchmark::DoNotOptimize(x);
      benchmark::DoNotOptimize(y);
      benchmark::DoNotOptimize(z);
      benchmark::DoNotOptimize(intensity);
      benchmark::DoNotOptimize(id);
      x = p.x;
      y = p.y;
      z = p.z;
      intensity = p.intensity;
      id = p.id;
    }
  }
}

BENCHMARK(BenchMsgWrapperAddPointToCloud);
BENCHMARK(BenchMsgWrapperResizeAndAddPointToCloud);
BENCHMARK(BenchMsgWrapperPushBackPointToCloud);


BENCHMARK(BenchMsgWrapperAccessPoint);
