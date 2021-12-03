// Copyright 2019 the Autoware Foundation
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
#include <ndt/utils.hpp>
#include <Eigen/LU>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <vector>
#include <limits>
#include <string>
#include "test_ndt_map.hpp"
#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using autoware::localization::ndt::DynamicNDTVoxel;
using autoware::localization::ndt::StaticNDTVoxel;
using autoware::localization::ndt::Real;
using autoware::localization::ndt::try_stabilize_covariance;
using autoware::localization::ndt::StaticNDTMap;
using autoware::perception::filters::voxel_grid::Config;
constexpr std::uint32_t DenseNDTMapContext::NUM_POINTS;

class DenseNDTMapTest : public DenseNDTMapContext, public ::testing::Test {};
class NDTMapTest : public NDTMapContext, public ::testing::Test
{
public:
  NDTMapTest()
  {
    // Make voxel grid's max point to include points up to 50.0
    m_max_point.x = NUM_POINTS + 0.5F;
    m_max_point.y = NUM_POINTS + 0.5F;
    m_max_point.z = NUM_POINTS + 0.5F;
  }
};

TEST(DynamicNDTVoxelTest, NdtDenseVoxelBasicIo) {
  constexpr auto eps = 1e-6;

  DynamicNDTVoxel voxel;

  EXPECT_THROW(voxel.centroid(), std::out_of_range);
  EXPECT_THROW(voxel.covariance(), std::out_of_range);
  EXPECT_EQ(voxel.count(), 0U);
  Eigen::Vector3d point({5, 5, 5});

  voxel.add_observation(point);

  // voxel is considered unoccupied when it has less point than its point threshold
  if (DynamicNDTVoxel::NUM_POINT_THRESHOLD > 1U) {
    EXPECT_THROW(voxel.centroid(), std::out_of_range);
    EXPECT_THROW(voxel.covariance(), std::out_of_range);
  }

  // Add the same point until the voxel has sufficient number of points
  for (uint64_t i = 1U; i < DynamicNDTVoxel::NUM_POINT_THRESHOLD; i++) {
    EXPECT_THROW(voxel.covariance(), std::out_of_range);
    voxel.add_observation(point);
  }

  EXPECT_EQ(voxel.count(), DynamicNDTVoxel::NUM_POINT_THRESHOLD);
  // Centroid should equal to the point as we added the same point multiple times.
  EXPECT_TRUE(point.isApprox(voxel.centroid(), eps));

  // Covariance values are zero since all points are the same
  // and there's no variance.
  EXPECT_NO_THROW(
    EXPECT_LT(voxel.covariance().norm(), eps);
  );
}

///////////////////////////////////

TEST(DynamicNDTVoxelTest, NdtDenseVoxelBasic) {
  constexpr auto eps = 1e-6;
  DynamicNDTVoxel voxel;
  auto num_points = 5U;
  EXPECT_GE(num_points, DynamicNDTVoxel::NUM_POINT_THRESHOLD);

  std::vector<Eigen::Vector3d> points;
  // Add points to the voxel ([0,0,0]... to [4,4,4])
  for (auto i = 0U; i < num_points; i++) {
    auto point =
      Eigen::Vector3d{static_cast<float64_t>(i),
      static_cast<float64_t>(i), static_cast<float64_t>(i)};
    points.push_back(point);
    voxel.add_observation(point);
  }

  // validate the mean in numpy
  // np.mean(np.array([[0,1,2,3,4],[0,1,2,3,4],[0,1,2,3,4]]),1)
  Eigen::Vector3d expected_centroid{2.0, 2.0, 2.0};
  // Validate covariance in numpy: np.cov(np.array([[0,1,2,3,4],[0,1,2,3,4],[0,1,2,3,4]]))
  Eigen::Matrix3d expected_covariance;
  expected_covariance.setConstant(2.5);

  EXPECT_NO_THROW(
    EXPECT_TRUE(voxel.centroid().isApprox(expected_centroid, eps));
  );
  EXPECT_NO_THROW(
    EXPECT_TRUE(voxel.covariance().isApprox(expected_covariance, eps));
  );
}


TEST_F(DenseNDTMapTest, MapLookup) {
  constexpr auto eps = 1e-5;
  // The idea is to have a 5x5x5 grid with cell edge length of 1
  auto grid_config = Config(m_min_point, m_max_point, m_voxel_size, m_capacity);

  DynamicNDTMap ndt_map(grid_config);

  // No map is added, so a lookup should return an empty vector.
  EXPECT_TRUE(ndt_map.cell(0.0F, 0.0F, 0.0F).empty());

  // build a pointcloud map.
  // It contains 5*5*5*7 points where each cell would have a center
  // (ranging from (1,1,1) to (5,5,5))
  // and 6 surrounding points with a 0.3 distance from the center
  build_pc(grid_config);

  // The center points are added to a map with their voxel indices for easy lookup
  EXPECT_EQ(m_voxel_centers.size(), 125U);

  // Insert the pointcloud into the ndt map
  EXPECT_NO_THROW(ndt_map.insert(m_pc));
  // ndt map has 125 voxels now: a 5x5x5 grid
  EXPECT_EQ(ndt_map.size(), 125U);

  // Al cells have the same variance. The value can be validated via numpy:
  // >>> dev = 0.3
  // >>> np.cov(np.array([ [1, 1+dev, 1-dev,1,1,1,1], [1,1,1,1+dev,1-dev,1,1], [1,1,1,1,1,1+dev,
  // 1-dev]  ]))
  Eigen::Matrix3d expected_cov;
  expected_cov << 0.03, 0.0, 0.0,
    0.0, 0.03, 0.0,
    0.0, 0.0, 0.03;

  for (const auto & voxel_it : ndt_map) {
    Eigen::Vector3d center{0.0, 0.0, 0.0};
    // Each voxel has 7 points
    EXPECT_EQ(voxel_it.second.count(), 7U);
    EXPECT_NO_THROW(center = voxel_it.second.centroid());
    // Check if the voxel centroid is the same as the intended centroid
    auto voxel_idx = grid_config.index(center);
    EXPECT_TRUE(m_voxel_centers[voxel_idx].isApprox(center, eps));
    // Check if covariance matches the pre-computed value
    EXPECT_NO_THROW(
      EXPECT_TRUE(voxel_it.second.covariance().isApprox(expected_cov, eps));
    );
  }

  // Iterate the grid and do lookups:
  for (auto x = 1; x <= POINTS_PER_DIM; x++) {
    for (auto y = 1; y <= POINTS_PER_DIM; y++) {
      for (auto z = 1; z <= POINTS_PER_DIM; z++) {
        // Query the idx of the expected centroid via the config object:
        auto expected_idx = grid_config.index(Eigen::Vector3d(x, y, z));
        // Get the cell index ndt map estimated:
        auto cell = ndt_map.cell(
          static_cast<float32_t>(x),
          static_cast<float32_t>(y),
          static_cast<float32_t>(z));
        auto map_idx = grid_config.index(cell[0].centroid());
        EXPECT_EQ(expected_idx, map_idx);
      }
    }
  }
}

///////////////////////////////////////

TEST(StaticNDTVoxelTest, NdtMapVoxelBasics) {
  StaticNDTVoxel vx;
  Eigen::Vector3d pt{5.0, 5.0, 5.0};
  Eigen::Matrix3d icov;
  icov.setIdentity();
  icov(0, 0) = 7.0;
  icov(1, 1) = 17.0;
  icov(2, 2) = 3.0;
  // default constructor zero-initializes and doesn't set the voxel as occupied
  EXPECT_FALSE(vx.usable());
  EXPECT_THROW(vx.centroid(), std::out_of_range);
  EXPECT_THROW(vx.covariance(), std::out_of_range);

  StaticNDTVoxel vx2{pt, icov};
  EXPECT_TRUE(vx2.usable());
  EXPECT_TRUE(vx2.centroid().isApprox(pt, std::numeric_limits<Real>::epsilon()));
  EXPECT_TRUE(vx2.inverse_covariance().isApprox(icov, std::numeric_limits<Real>::epsilon()));

  // Copying is legal as we replace the voxels in the map via copy constructors.
  vx = vx2;
  EXPECT_TRUE(vx.usable());
  EXPECT_TRUE(vx.centroid().isApprox(pt, std::numeric_limits<Real>::epsilon()));
  EXPECT_TRUE(vx.inverse_covariance().isApprox(icov, std::numeric_limits<Real>::epsilon()));
}

TEST(StaticNDTVoxelTest, NdtMapVoxelInverseCovarianceBasic) {
  // Use a DynamicNDTVoxel to compute a valid covariance
  DynamicNDTVoxel generator_voxel;
  generator_voxel.add_observation(Eigen::Vector3d{1.0, 1.0, 7.0});
  generator_voxel.add_observation(Eigen::Vector3d{2.0, 9.0, 2.0});
  generator_voxel.add_observation(Eigen::Vector3d{0.0, 3.0, 5.0});
  generator_voxel.add_observation(Eigen::Vector3d{4.0, 5.0, 8.0});
  const auto & centroid = generator_voxel.centroid();
  bool8_t invertible{false};
  Eigen::Matrix3d covariance{};

  // test inverse_covariance() without stabilization
  auto inv_covariance_before_stabilization = generator_voxel.inverse_covariance();
  ASSERT_TRUE(inv_covariance_before_stabilization);  // optional should contain valid value
  inv_covariance_before_stabilization.value().computeInverseWithCheck(covariance, invertible);
  EXPECT_TRUE(generator_voxel.covariance().isApprox(covariance));

  // test inverse_covariance() with stabilization
  (void) generator_voxel.try_stabilize();
  auto inv_covariance_after_stabilizatoin = generator_voxel.inverse_covariance();
  ASSERT_TRUE(inv_covariance_after_stabilizatoin);  // optional should contain valid value
  StaticNDTVoxel voxel(centroid, inv_covariance_after_stabilizatoin.value());
  ASSERT_TRUE(try_stabilize_covariance(inv_covariance_after_stabilizatoin.value()));
  inv_covariance_after_stabilizatoin.value().computeInverseWithCheck(covariance, invertible);
  ASSERT_TRUE(invertible);
  EXPECT_TRUE(covariance.isApprox(voxel.covariance()));
  EXPECT_TRUE(voxel.usable());
}

TEST_F(NDTMapTest, MapRepresentationBadInput) {
  sensor_msgs::msg::PointCloud2 invalid_pc1;
  sensor_msgs::msg::PointCloud2 invalid_pc2;

  // initialize the messages
  // Message with the missing fields
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> pc_view_with_wrong_point{
    invalid_pc1, "map"};
  pc_view_with_wrong_point.resize(100U);
  // Correct message format, but empty
  autoware::localization::ndt::NdtMapCloudModifier empty_view{invalid_pc2, "map"};

  StaticNDTMap map_grid{};

  EXPECT_THROW(map_grid.set(invalid_pc1), std::runtime_error);
  EXPECT_THROW(map_grid.set(invalid_pc2), std::runtime_error);
}

TEST_F(NDTMapTest, MapRepresentationBasics) {
  sensor_msgs::msg::PointCloud2 msg;
  autoware::localization::ndt::NdtMapCloudModifier modifier{msg, "map"};
  auto add_pt = [&msg, &modifier](Real value) {
      modifier.push_back({value, value, value, value, 0.0, 0.0, value, 0.0, value});
    };


  auto grid_config = Config(m_min_point, m_max_point, m_voxel_size, m_capacity);

  // This grid will be used to generate
  DynamicNDTMap generator_grid(grid_config);
  StaticNDTMap map_grid{};

  // No map is added, so a lookup should result in an exception
  EXPECT_THROW(map_grid.cell(0.0, 0.0, 0.0), std::runtime_error);

  add_pt(grid_config.get_min_point().x);
  add_pt(grid_config.get_max_point().x);
  add_pt(grid_config.get_voxel_size().x);

  auto value = Real{1.0};
  for (auto i = 0U; i < NUM_POINTS; ++i) {
    const Eigen::Vector3d added_pt{value, value, value};
    // Turn the point into a pointcloud to insert. The point should be inserted enough to make
    // the voxel usable. (equal to NUM_POINT_THRESHOLD)
    generator_grid.insert(
      make_pcl(
        std::vector<Eigen::Vector3d>{
      DynamicNDTVoxel::NUM_POINT_THRESHOLD, added_pt}));

    // For simplicity,
    // the inserted points already correspond to the centroids and there's point per voxel.
    const auto & generating_voxel = generator_grid.cell(added_pt)[0U];
    ASSERT_TRUE(
      generating_voxel.centroid().isApprox(
        added_pt,
        std::numeric_limits<Real>::epsilon()));
    ASSERT_EQ(generating_voxel.get().count(), DynamicNDTVoxel::NUM_POINT_THRESHOLD);

    add_pt(value);

    ++value;
  }

  // Each point should correspond to a single voxel for this test case
  ASSERT_EQ(generator_grid.size(), NUM_POINTS);

  // All points should be able to be inserted since
  EXPECT_NO_THROW(map_grid.set(msg));
  EXPECT_EQ(map_grid.size(), generator_grid.size());

  // dif to be used for grid lookup.
  auto diff = 0.1;
  // Ensure, when added, the diff doesn't drift a centroid to another voxel.
  ASSERT_LT(diff, grid_config.get_voxel_size().x);

  // Check if every voxel in the generator grid is passed to the map representation.
  for (auto & vx : generator_grid) {
    EXPECT_NE(
      std::find_if(
        map_grid.begin(), map_grid.end(), [&vx](const auto & map_elem) {
          return map_elem.first == vx.first;
        }), map_grid.end());

    auto pt = vx.second.centroid();
    // slightly move the point before looking up to check if it gets the correct voxel.
    pt(0) += diff;
    EXPECT_TRUE(
      map_grid.cell(pt(0), pt(1), pt(2))[0].centroid().isApprox(
        vx.second.centroid(),
        std::numeric_limits<Real>::epsilon()));
  }

  map_grid.clear();
  EXPECT_EQ(map_grid.size(), 0U);
}


///////////////////////////// Function definitions:

sensor_msgs::msg::PointCloud2 make_pcl(
  const std::vector<sensor_msgs::msg::PointField> & fields,
  uint32_t height,
  uint32_t data_size,
  uint32_t row_step,
  uint32_t width,
  uint32_t point_step)
{
  sensor_msgs::msg::PointCloud2 msg;

  msg.data.resize(data_size, uint8_t{0U});
  msg.fields = fields;
  msg.row_step = row_step;
  msg.height = height;
  msg.width = width;
  msg.point_step = point_step;
  return msg;
}

sensor_msgs::msg::PointCloud2 make_pcl(const std::vector<Eigen::Vector3d> & pts)
{
  sensor_msgs::msg::PointCloud2 cloud;
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> modifier{cloud, "map"};
  for (const auto & pt : pts) {
    autoware::common::types::PointXYZI ptF{};
    ptF.x = static_cast<float>(pt(0U));
    ptF.y = static_cast<float>(pt(1U));
    ptF.z = static_cast<float>(pt(2U));
    modifier.push_back(ptF);
  }
  return cloud;
}


sensor_msgs::msg::PointField make_pf(
  std::string name, uint32_t offset, uint8_t datatype,
  uint32_t count)
{
  sensor_msgs::msg::PointField pf;
  pf.name = name;
  pf.offset = offset;
  pf.datatype = datatype;
  pf.count = count;
  return pf;
}

autoware::common::types::PointXYZI get_point_from_vector(const Eigen::Vector3d & v)
{
  autoware::common::types::PointXYZI ptF{};
  ptF.x = static_cast<float32_t>(v(0));
  ptF.y = static_cast<float32_t>(v(1));
  ptF.z = static_cast<float32_t>(v(2));
  return ptF;
}

// add the point `center` and 4 additional points in a fixed distance from the center
// resulting in 7 points with random but bounded covariance
void add_cell(
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> & msg_wrapper,
  const Eigen::Vector3d & center, float64_t fixed_deviation)
{
  msg_wrapper.push_back(get_point_from_vector(center));

  std::vector<Eigen::Vector3d> points;
  for (auto idx = 0U; idx < 3U; idx++) {
    for (auto mode = 0u; mode < 2u; mode++) {
      auto deviated_pt = center;
      if (mode == 0U) {
        deviated_pt(idx) += fixed_deviation;
      } else {
        deviated_pt(idx) -= fixed_deviation;
      }
      points.push_back(deviated_pt);
      msg_wrapper.push_back(get_point_from_vector(deviated_pt));
    }
  }
}

DenseNDTMapContext::DenseNDTMapContext()
: m_pc_wrapper{m_pc, "map"}
{
  // Grid and spatial hash uses these boundaries. The setup allows for a grid of 125 cells: 5x5x5
  // where the centroid coordinates range from the integers 1 to 5 and the voxel size is 1
  m_min_point.x = 0.5F;
  m_min_point.y = 0.5F;
  m_min_point.z = 0.5F;
  m_max_point.x = 5.5F;
  m_max_point.y = 5.5F;
  m_max_point.z = 5.5F;
  m_voxel_size.x = 1.0F;
  m_voxel_size.y = 1.0F;
  m_voxel_size.z = 1.0F;
}
void DenseNDTMapContext::build_pc(const Config & cfg)
{
  for (auto x = 1U; x <= POINTS_PER_DIM; ++x) {
    for (auto y = 1U; y <= POINTS_PER_DIM; ++y) {
      for (auto z = 1U; z <= POINTS_PER_DIM; ++z) {
        Eigen::Vector3d center{static_cast<float64_t>(x), static_cast<float64_t>(y),
          static_cast<float64_t>(z)};
        add_cell(m_pc_wrapper, center, FIXED_DEVIATION);
        m_voxel_centers[cfg.index(center)] = center;
      }
    }
  }
}
