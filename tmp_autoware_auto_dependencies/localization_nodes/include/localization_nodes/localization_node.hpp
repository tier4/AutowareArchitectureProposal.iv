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

#ifndef LOCALIZATION_NODES__LOCALIZATION_NODE_HPP_
#define LOCALIZATION_NODES__LOCALIZATION_NODE_HPP_

#include <localization_common/optimized_registration_summary.hpp>
#include <localization_common/initialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <time_utils/time_utils.hpp>

#include <autoware_utils/geometry/geometry.hpp>

#include <helper_functions/message_adapters.hpp>
#include <localization_nodes/visibility_control.hpp>
#include <localization_nodes/constraints.hpp>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <chrono>

namespace autoware
{
namespace localization
{
namespace localization_nodes
{
using common::helper_functions::message_field_adapters::get_frame_id;
using common::helper_functions::message_field_adapters::get_stamp;

/// Helper struct that groups topic name and QoS setting for a publisher or subscription
struct TopicQoS
{
  std::string topic;
  rclcpp::QoS qos;
};

/// Enum to specify if the localizer node must publish to `/tf` topic or not
enum class LocalizerPublishMode
{
  PUBLISH_TF,
  NO_PUBLISH_TF
};

template <class Rep, class Period>
double as_microsecond(const std::chrono::duration<Rep,Period> & d)
{
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(d).count());
}

/// Base relative localizer node that publishes map->base_link relative
/// transform messages for a given observation source and map.
/// \tparam ObservationMsgT Message type to register against a map.
/// \tparam MapMsgT Map type
/// \tparam LocalizerT Localizer type.
/// \tparam PoseInitializerT Pose initializer type.
template<typename ObservationMsgT,
  typename MapMsgT,
  typename MapT,
  typename LocalizerT,
  typename PoseInitializerT,
  Requires = traits::LocalizerConstraint<LocalizerT, ObservationMsgT, MapT>::value,
  Requires = traits::MapConstraint<MapT, MapMsgT>::value>
class LOCALIZATION_NODES_PUBLIC RelativeLocalizerNode : public rclcpp::Node
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  using RegistrationSummary = localization_common::OptimizedRegistrationSummary;
  // During the experiments, it was found out that running the `tf_listener` in parallel
  // resulted in more robust ndt initialization performance: #868
  static constexpr bool USE_DEDICATED_TF_THREAD{true};

  /// Constructor
  /// \param node_name Name of node
  /// \param name_space Namespace of node
  /// \param observation_sub_config topic and QoS setting for the observation subscription.
  /// \param map_sub_config topic and QoS setting for the map subscription.
  /// \param pose_pub_config topic and QoS setting for the output pose publisher.
  /// \param initial_pose_sub_config topic and QoS setting for the initialpose subscription.
  /// \param pose_initializer Pose initializer.
  /// \param publish_tf Whether to publish to the `tf` topic. This can be used to publish transform
  /// messages when the relative localizer is the only source of localization.
  RelativeLocalizerNode(
    const std::string & node_name, const std::string & name_space,
    const TopicQoS & observation_sub_config,
    const TopicQoS & map_sub_config,
    const TopicQoS & pose_pub_config,
    const TopicQoS & initial_pose_sub_config,
    const PoseInitializerT & pose_initializer,
    LocalizerPublishMode publish_tf = LocalizerPublishMode::NO_PUBLISH_TF)
  : Node(node_name, name_space),
    m_pose_initializer(pose_initializer),
    m_tf_listener(m_tf_buffer, USE_DEDICATED_TF_THREAD),
    m_observation_sub(create_subscription<ObservationMsgT>(
        observation_sub_config.topic,
        observation_sub_config.qos,
        [this](typename ObservationMsgT::ConstSharedPtr msg) {observation_callback(msg);})),
    m_map_sub(
      create_subscription<MapMsgT>(
        map_sub_config.topic, map_sub_config.qos,
        [this](typename MapMsgT::ConstSharedPtr msg) {map_callback(msg);})),
    m_pose_publisher(
      create_publisher<PoseWithCovarianceStamped>(
        pose_pub_config.topic,
        pose_pub_config.qos)),
    m_initial_pose_sub(
      create_subscription<PoseWithCovarianceStamped>(
        initial_pose_sub_config.topic, initial_pose_sub_config.qos,
        [this](const typename PoseWithCovarianceStamped::ConstSharedPtr msg) {
          initial_pose_callback(msg);
        })) {
    if (publish_tf == LocalizerPublishMode::PUBLISH_TF) {
      m_tf_publisher = create_publisher<tf2_msgs::msg::TFMessage>("/tf", pose_pub_config.qos);
    }
  }

  // Constructor for ros2 components
  // TODO(yunus.caliskan): refactor constructors together reduce the repeated code.
  RelativeLocalizerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options,
    const PoseInitializerT & pose_initializer)
  : Node(node_name, options),
    m_pose_initializer(pose_initializer),
    m_tf_listener(m_tf_buffer, USE_DEDICATED_TF_THREAD),
    m_observation_sub(create_subscription<ObservationMsgT>(
        "points_in",
        rclcpp::SensorDataQoS{rclcpp::KeepLast{
            static_cast<size_t>(declare_parameter("observation_sub.history_depth").template
            get<size_t>())}},
        [this](typename ObservationMsgT::ConstSharedPtr msg) {observation_callback(msg);})),
    m_map_sub(
      create_subscription<MapMsgT>(
        "ndt_map",
        rclcpp::QoS{rclcpp::KeepLast{
            static_cast<size_t>(declare_parameter("map_sub.history_depth").
            template get<size_t>())}}.transient_local(),
        [this](typename MapMsgT::ConstSharedPtr msg) {map_callback(msg);})),
    m_pose_publisher(
      create_publisher<PoseWithCovarianceStamped>(
        "ndt_pose",
        rclcpp::QoS{rclcpp::KeepLast{
            static_cast<size_t>(declare_parameter(
              "pose_pub.history_depth").template get<size_t>())}})),
    m_initial_pose_sub(
      create_subscription<PoseWithCovarianceStamped>(
        "initialpose",
        rclcpp::QoS{rclcpp::KeepLast{10}},
        [this](const typename PoseWithCovarianceStamped::ConstSharedPtr msg) {
          initial_pose_callback(msg);
        }))
  {
    init();
  }

  /// Constructor using ros parameters
  /// \param node_name Node name
  /// \param name_space Node namespace
  /// \param pose_initializer Pose initializer
  RelativeLocalizerNode(
    const std::string & node_name, const std::string & name_space,
    const PoseInitializerT & pose_initializer)
  : Node(node_name, name_space),
    m_pose_initializer(pose_initializer),
    m_tf_listener(m_tf_buffer, USE_DEDICATED_TF_THREAD),
    m_observation_sub(create_subscription<ObservationMsgT>(
        "points_in",
        rclcpp::SensorDataQoS{rclcpp::KeepLast{
            static_cast<size_t>(declare_parameter("observation_sub.history_depth").template
            get<size_t>())}},
        [this](typename ObservationMsgT::ConstSharedPtr msg) {observation_callback(msg);})),
    m_map_sub(
      create_subscription<MapMsgT>(
        "ndt_map",
        rclcpp::QoS{rclcpp::KeepLast{
            static_cast<size_t>(declare_parameter("map_sub.history_depth").
            template get<size_t>())}}.transient_local(),
        [this](typename MapMsgT::ConstSharedPtr msg) {map_callback(msg);})),
    m_pose_publisher(
      create_publisher<PoseWithCovarianceStamped>(
        "ndt_pose",
        rclcpp::QoS{rclcpp::KeepLast{
            static_cast<size_t>(declare_parameter(
              "pose_pub.history_depth").template get<size_t>())}})),
    m_initial_pose_sub(
      create_subscription<PoseWithCovarianceStamped>(
        "initialpose",
        rclcpp::QoS{rclcpp::KeepLast{10}},
        [this](const typename PoseWithCovarianceStamped::ConstSharedPtr msg) {
          initial_pose_callback(msg);
        }))
  {
    init();
  }

  /// Get a const pointer of the output publisher. Can be used for matching against subscriptions.
  const typename rclcpp::Publisher<PoseWithCovarianceStamped>::ConstSharedPtr get_publisher()
  {
    return m_pose_publisher;
  }

protected:
  /// Set the localizer.
  /// \param localizer_ptr rvalue to the localizer to set.
  void set_localizer(std::unique_ptr<LocalizerT> && localizer_ptr)
  {
    m_localizer_ptr = std::forward<std::unique_ptr<LocalizerT>>(localizer_ptr);
  }

  void set_map(std::unique_ptr<MapT> && map_ptr)
  {
    m_map_ptr = std::forward<std::unique_ptr<MapT>>(map_ptr);
  }

  /// Handle the exceptions during registration.
  virtual void on_bad_registration(std::exception_ptr eptr) // NOLINT
  {
    on_exception(eptr, "on_bad_registration");
  }

  /// Handle the exceptions during map setting.
  virtual void on_bad_map(std::exception_ptr eptr) // NOLINT
  {
    on_exception(eptr, "on_bad_map");
  }

  void on_exception(std::exception_ptr eptr, const std::string & error_source)  // NOLINT
  {
    try {
      if (eptr) {
        std::rethrow_exception(eptr);
      } else {
        RCLCPP_ERROR(get_logger(), std::string(error_source + ": error nullptr").c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), e.what());
    }
  }

  /// Default behavior when an observation is received with no valid existing map.
  virtual void on_observation_with_invalid_map(typename ObservationMsgT::ConstSharedPtr)
  {
    RCLCPP_WARN(
      get_logger(), "Received observation without a valid map, "
      "ignoring the observation.");
  }

  /// Default behavior when hte pose output is evaluated to be invalid.
  /// \param pose Pose output.
  virtual void on_invalid_output(const PoseWithCovarianceStamped & pose)
  {
    (void) pose;
    RCLCPP_WARN(
      get_logger(), "Relative localizer has an invalid pose estimate. "
      "The result is ignored.");
  }


  /// Validate the pose estimate given the registration summary and the initial guess.
  /// This function by default returns true.
  /// \param summary Registration summary.
  /// \param pose Pose estimate.
  /// \param guess Initial guess.
  /// \return True if the estimate is valid and can be published.
  virtual bool validate_output(
    const RegistrationSummary & summary,
    const PoseWithCovarianceStamped & pose, const TransformStamped & guess)
  {
    (void) summary;
    (void) pose;
    (void) guess;
    return true;
  }

private:
  /// Check the pointer and throw if null.
  template<typename PtrT>
  void assert_ptr_not_null(const PtrT & ptr, const std::string & name) const
  {
    if (!ptr) {
      throw std::runtime_error(
              name + " pointer is null. Make sure it is properly initialized before using.");
    }
  }

  void init()
  {
    if (declare_parameter("publish_tf").template get<bool>()) {
      m_tf_publisher = create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf",
        rclcpp::QoS{rclcpp::KeepLast{m_pose_publisher->get_queue_size()}});
    }
    if (declare_parameter("load_initial_pose_from_parameters", false)) {
      m_pose_initializer.set_fallback_pose(get_initial_pose());
    }
  }

  geometry_msgs::msg::TransformStamped get_initial_pose()
  {
    geometry_msgs::msg::TransformStamped initial_transform;
    auto & tf = initial_transform.transform;
    tf.rotation.x = declare_parameter("initial_pose.quaternion.x").template get<float64_t>();
    tf.rotation.y = declare_parameter("initial_pose.quaternion.y").template get<float64_t>();
    tf.rotation.z = declare_parameter("initial_pose.quaternion.z").template get<float64_t>();
    tf.rotation.w = declare_parameter("initial_pose.quaternion.w").template get<float64_t>();
    tf.translation.x = declare_parameter("initial_pose.translation.x").template get<float64_t>();
    tf.translation.y = declare_parameter("initial_pose.translation.y").template get<float64_t>();
    tf.translation.z = declare_parameter("initial_pose.translation.z").template get<float64_t>();
    initial_transform.header.frame_id = "map";
    initial_transform.child_frame_id = "base_link";
    return initial_transform;
  }


/// Process the registration summary. By default does nothing.
  virtual void handle_registration_summary(const RegistrationSummary &) {}

  /// Callback that registers each received observation and outputs the result.
  /// \param msg_ptr Pointer to the observation message.
  void observation_callback(typename ObservationMsgT::ConstSharedPtr msg_ptr)
  {
    const auto exe_start_time = std::chrono::system_clock::now();

    // Check to ensure the pointers are initialized.
    assert_ptr_not_null(m_localizer_ptr, "localizer");
    assert_ptr_not_null(m_map_ptr, "map");

    if (!m_map_ptr->valid()) {
      on_observation_with_invalid_map(msg_ptr);
      return;
    }

    const auto observation_time = ::time_utils::from_message(get_stamp(*msg_ptr));
    const auto & observation_frame = get_frame_id(*msg_ptr);
    const auto & map_frame = m_map_ptr->frame_id();

    try {
      geometry_msgs::msg::TransformStamped initial_guess = m_pose_initializer.guess(
        m_tf_buffer, observation_time, map_frame, observation_frame);
      RegistrationSummary summary{};
      const auto pose_out =
        m_localizer_ptr->register_measurement(*msg_ptr, initial_guess, *m_map_ptr, &summary);
      if (validate_output(summary, pose_out, initial_guess)) {
        m_pose_publisher->publish(pose_out);
        // This is to be used when no state estimator or alternative source of
        // localization is available.
        if (m_tf_publisher) {
          publish_tf(pose_out);
          // republish point cloud so visualization has no issues with the timestamp
          // being too new (no transform yet). Reset the timestamp to zero so visualization
          // is not bothered if odom->base_link transformation is available
          // only at different time stamps.
          auto msg = *msg_ptr;
          msg.header.stamp = time_utils::to_message(tf2::TimePointZero);
          m_obs_republisher->publish(msg);
        }

        handle_registration_summary(summary);
      } else {
        on_invalid_output(pose_out);
      }
    } catch (...) {
      on_bad_registration(std::current_exception());
    }
    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time = as_microsecond(exe_end_time - exe_start_time) / 1000.0;
    std::cerr << "exe_time: " << exe_time << std::endl;
  }

  /// Callback that updates the map.
  /// \param msg_ptr Pointer to the map message.
  void map_callback(typename MapMsgT::ConstSharedPtr msg_ptr)
  {
    assert_ptr_not_null(m_map_ptr, "map");
    try {
      m_map_ptr->set(*msg_ptr);
    } catch (...) {
      on_bad_map(std::current_exception());
    }
  }

  geometry_msgs::msg::TransformStamped get_odom(const builtin_interfaces::msg::Time & pose_stamp)
  {
    try {
      return m_tf_buffer.lookupTransform("odom", "base_link", time_utils::from_message(pose_stamp));
    } catch (const tf2::ExtrapolationException &) {
      return m_tf_buffer.lookupTransform("odom", "base_link", tf2::TimePointZero);
    }
  }

  /// Publish the pose message as a transform.
  void publish_tf(const PoseWithCovarianceStamped & pose_msg)
  {
    const auto & pose = pose_msg.pose.pose;
    const auto & map_frame_id = m_map_ptr->frame_id();
    tf2::Quaternion rotation{pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w};
    tf2::Vector3 translation{pose.position.x, pose.position.y, pose.position.z};
    const tf2::Transform map_base_link_transform{rotation, translation};

    // Wait for odom to base_link transform to be available
    bool odom_to_bl_found = m_tf_buffer.canTransform("odom", "base_link", tf2::TimePointZero);

    while (!odom_to_bl_found) {
      RCLCPP_INFO(get_logger(), "Waiting for odom to base_link transform to be available.");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      odom_to_bl_found = m_tf_buffer.canTransform("odom", "base_link", tf2::TimePointZero);
    }

    const geometry_msgs::msg::TransformStamped odom_tf = get_odom(pose_msg.header.stamp);
    tf2::Transform odom_base_link_transform;
    tf2::fromMsg(odom_tf.transform, odom_base_link_transform);

    const auto map_odom_tf = map_base_link_transform * odom_base_link_transform.inverse();

    tf2_msgs::msg::TFMessage tf_message;
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = pose_msg.header.stamp;
    tf_stamped.header.frame_id = map_frame_id;
    tf_stamped.child_frame_id = "odom";
    tf_stamped.transform = tf2::toMsg(map_odom_tf);
    tf_message.transforms.push_back(tf_stamped);
    m_tf_publisher->publish(tf_message);
  }

  void initial_pose_callback(const typename PoseWithCovarianceStamped::ConstSharedPtr msg_ptr)
  {
    // The child frame is implicitly base_link.
    // Ensure the parent frame is the map frame
    assert_ptr_not_null(m_map_ptr, "map");
    const std::string & map_frame = m_map_ptr->frame_id();
    if (!m_tf_buffer.canTransform(map_frame, msg_ptr->header.frame_id, tf2::TimePointZero)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to find transform from %s to %s frame. Failed to give initial pose.",
        msg_ptr->header.frame_id.c_str(), map_frame.c_str());
      return;
    }
    const auto transform = m_tf_buffer.lookupTransform(
      map_frame, msg_ptr->header.frame_id,
      tf2::TimePointZero);

    geometry_msgs::msg::TransformStamped input_pose_stamped;
    input_pose_stamped.header = msg_ptr->header;
    input_pose_stamped.child_frame_id = "base_link";
    input_pose_stamped.transform = autoware_utils::pose2transform(msg_ptr->pose.pose);
    geometry_msgs::msg::TransformStamped transformed_pose_stamped;
    tf2::doTransform(input_pose_stamped, transformed_pose_stamped, transform);

    // Note: The frame_id in the result is already the map_frame, no need to set it.
    transformed_pose_stamped.child_frame_id = "base_link";

    // For future reference: We can't write this pose to /tf here.
    // If this message comes from RViz and we're running in simulation, RViz
    // and data coming from LGSVL (the observations) will sometimes have very
    // large differences in their timestamps (e.g. RViz being 40s newer).
    // If this pose was published to /tf, it would be seen as being way in the
    // future and the localizer couldn't use it as its next initial pose.
    // We'd need to know the current time before it can be published, and set the
    // time in the header to a recent time.
    m_pose_initializer.set_fallback_pose(transformed_pose_stamped);
  }

  std::unique_ptr<LocalizerT> m_localizer_ptr;
  std::unique_ptr<MapT> m_map_ptr;
  PoseInitializerT m_pose_initializer;
  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
  typename rclcpp::Subscription<ObservationMsgT>::SharedPtr m_observation_sub;
  typename rclcpp::Subscription<MapMsgT>::SharedPtr m_map_sub;
  typename rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr m_pose_publisher;
  typename rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_publisher{nullptr};
  typename rclcpp::Publisher<ObservationMsgT>::SharedPtr m_obs_republisher{
    create_publisher<ObservationMsgT>("observation_republish", 10)};

  // Receive updates from "/initialpose" (e.g. rviz2)
  typename rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr m_initial_pose_sub;
};

template<typename ObservationMsgT, typename MapMsgT, typename MapT, typename LocalizerT,
  typename PoseInitializerT, Requires R1, Requires R2>
constexpr bool RelativeLocalizerNode<ObservationMsgT, MapMsgT, MapT,
  LocalizerT, PoseInitializerT, R1, R2>::USE_DEDICATED_TF_THREAD;
}  // namespace localization_nodes
}  // namespace localization
}  // namespace autoware
#endif  // LOCALIZATION_NODES__LOCALIZATION_NODE_HPP_
