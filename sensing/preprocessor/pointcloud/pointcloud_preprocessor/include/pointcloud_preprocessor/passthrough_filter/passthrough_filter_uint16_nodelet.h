#pragma once

#include <pcl/search/pcl_search.h>
#include "pointcloud_preprocessor/passthrough_filter/passthrough_uint16.h"
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor {
class PassThroughFilterUInt16Component : public pointcloud_preprocessor::Filter {
 protected:
  virtual void filter(const PointCloud2ConstPtr& input, const IndicesPtr& indices, PointCloud2& output);

  // void config_callback(pointcloud_preprocessor::PassThroughFilterUInt16Config& config, uint32_t level);

 private:
  pcl::PassThroughUInt16<pcl::PCLPointCloud2> impl_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PassThroughFilterUInt16Component(const rclcpp::NodeOptions &options);
};
}  // namespace pointcloud_preprocessor
