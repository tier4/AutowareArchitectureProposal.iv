// Copyright 2019-2020 the Autoware Foundation
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


#ifndef LOCALIZATION_COMMON__INITIALIZATION_HPP_
#define LOCALIZATION_COMMON__INITIALIZATION_HPP_

#include <localization_common/visibility_control.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <helper_functions/crtp.hpp>
#include <time_utils/time_utils.hpp>
#include <tf2/buffer_core.h>
#include <experimental/optional>
#include <string>

// probably include the motion model

namespace autoware
{
namespace localization
{
namespace localization_common
{

/// The Pose initializer helps initialize relative localizer algorithms with an initial guess.
/// Extrapolation policy must be defined within the implementation class.
/// \tparam Derived CRTP implementation class
template<typename Derived>
class LOCALIZATION_COMMON_PUBLIC PoseInitializerBase
  : public common::helper_functions::crtp<Derived>
{
  using PoseT = geometry_msgs::msg::TransformStamped;

public:
  /// Guess the pose at a given time point. This function will look the transform up in the
  /// transform graph between the specified frames. If extrapolation is required, the behavior is
  /// determined by the implementation class. tf2 lookup may generate exceptions if the lookup
  /// fails in other ways. For details, see tf2::BufferCore class.
  /// If a fallback pose was set externally, then the first lookup failure is ignored and the
  /// fallback pose is served instead.
  /// \param tf_graph Transform graph that contains all the transforms to look up.
  /// \param time_point Time to guess the pose.
  /// \param target_frame Target frame of the transform. (i.e. "map")
  /// \param source_frame Source frame of the transform. (i.e. "base_link")
  /// \return The transform at the given time point
  PoseT guess(
    const tf2::BufferCore & tf_graph, tf2::TimePoint time_point,
    const std::string & target_frame, const std::string & source_frame)
  {
    try {
      // attempt to get transform at a given point.
      return tf_graph.lookupTransform(target_frame, source_frame, time_point);
      // TODO(yunus.caliskan): Consider detecting too large interpolations and issuing a
      //  warning/error.
    } catch (const tf2::ExtrapolationException &) {
      return this->impl().extrapolate(tf_graph, time_point, target_frame, source_frame);
    } catch (...) {
      if (!m_fallback_pose) {
        std::rethrow_exception(std::current_exception());
      }
      if ((m_fallback_pose->header.frame_id != target_frame) ||
        (m_fallback_pose->child_frame_id != source_frame))
      {
        throw std::runtime_error(
                "The initial pose provided to the pose initializer does not "
                "have the matching frame IDs.");
      }
      m_fallback_pose.value().header.stamp = ::time_utils::to_message(time_point);
      return m_fallback_pose.value();
    }
  }

  /// \brief Explicitly set a transform to be served first time a queried transform is not
  /// available. This pose is only served once and after that, it is not available until a
  /// new one is set.
  /// \param pose Fallback pose to set
  void set_fallback_pose(const PoseT & pose)
  {
    m_fallback_pose.emplace(pose);
  }

private:
  std::experimental::optional<PoseT> m_fallback_pose{std::experimental::nullopt};
};

/// Pose initialization implementation where the extrapolation policy is to simply
/// use the latest available transform.
class LOCALIZATION_COMMON_PUBLIC BestEffortInitializer
  : public PoseInitializerBase<BestEffortInitializer>
{
  using PoseT = geometry_msgs::msg::TransformStamped;

public:
  ///  Get the latest available transform.
  /// \param tf_graph Transform graph that contains all the transforms to look up.
  /// \param time_point Time to guess the pose.
  /// \param target_frame Target frame of the transform. (i.e. "base_link")
  /// \param source_frame Source frame of the transform. (i.e. "map")
  /// \return The transform at the given time point
  PoseT extrapolate(
    const tf2::BufferCore & tf_graph, tf2::TimePoint time_point,
    const std::string & target_frame, const std::string & source_frame);
};

}  // namespace localization_common
}  // namespace localization
}  // namespace autoware

#endif  // LOCALIZATION_COMMON__INITIALIZATION_HPP_
