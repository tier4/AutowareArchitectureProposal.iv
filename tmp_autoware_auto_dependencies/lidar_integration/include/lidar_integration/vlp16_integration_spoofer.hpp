// Copyright 2017-2018 the Autoware Foundation
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

#ifndef LIDAR_INTEGRATION__VLP16_INTEGRATION_SPOOFER_HPP_
#define LIDAR_INTEGRATION__VLP16_INTEGRATION_SPOOFER_HPP_

#include <common/types.hpp>
#include <lidar_integration/visibility_control.hpp>
#include <lidar_integration/udp_sender.hpp>
#include <thread>
#include <atomic>

namespace lidar_integration
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

class LIDAR_INTEGRATION_PUBLIC Vlp16IntegrationSpoofer
{
public:
  Vlp16IntegrationSpoofer(
    const char8_t * const ip,
    const uint16_t port,
    const float32_t rpm);
  ~Vlp16IntegrationSpoofer();

  void start();

  void stop();

  const uint32_t & send_count() const {return m_spoofer.send_count();}

  /// rpm min speed
  static constexpr float32_t MIN_RPM = 300.0F;
  /// rpm max speed
  static constexpr float32_t MAX_RPM = 1200.0F;

private:
  class SpoofTask
  {
public:
    SpoofTask(
      const char8_t * const ip,
      const uint16_t port,
      const float32_t rpm,
      const std::atomic_bool & running);

    const uint32_t & send_count() const {return m_send_count;}

    void start()
    {
      m_thread = std::thread{[this] {task_function();}};
    }

    void stop()
    {
      if (m_thread.joinable()) {
        m_thread.join();
      }
    }

protected:
    void task_function();

private:
    static constexpr uint32_t RAY_SIZE = 16U;
    // SENSOR SPECIFIC CONSTANTS
    /// resolution of azimuth angle: number of points in a full rotation
    static constexpr uint16_t AZIMUTH_ROTATION_RESOLUTION = 36000U;
    /// conversion from a degree (vlp) to idx
    static constexpr float32_t DEG2IDX =
      static_cast<float32_t>(AZIMUTH_ROTATION_RESOLUTION) / 360.0F;
    /// how intensity is quantized: 1 byte = 256 possible values
    static constexpr uint32_t NUM_INTENSITY_VALUES = 256U;

    /// All of these hardcoded values should remain fixed unless the VLP16 packet spec changes ///
    /// number of data blocks per data packet
    static constexpr uint16_t NUM_BLOCKS_PER_PACKET = 12U;
    /// number of points stored in a data block
    static constexpr uint16_t NUM_POINTS_PER_BLOCK = 32U;

    /// full (16 point) fire sequence takes this long to cycle
    static constexpr float32_t FIRE_SEQ_OFFSET_US = 55.296F;
    /// one laser fires for this long
    static constexpr float32_t FIRE_DURATION_US = 2.304F;

    /// \brief corresponds to an individual laser's firing and return
    /// First two bytes are distance, last byte is intensity
    struct DataChannel
    {
      uint8_t data[3U];
    };

    /// \brief corresponds to a vlp16 data block, which represents two full firings
    struct DataBlock
    {
      uint8_t flag[2U];
      uint8_t azimuth_bytes[2U];
      DataChannel channels[NUM_POINTS_PER_BLOCK];
    };

    /// \brief stores a velodyne data packet
    struct Packet
    {
      DataBlock blocks[NUM_BLOCKS_PER_PACKET];
      uint8_t timestamp_bytes[4U];
      uint8_t factory_bytes[2U];
    };

    static void uint16_to_bytes(const uint16_t val, uint8_t arr[]);

    static uint16_t bytes_to_uint16(const uint8_t arr[]);

    static void update_packet_azimuth(Packet & pkt, const uint16_t dth_tic);

    void init_packet(
      Packet & pkt,
      const float32_t rpm,
      const uint16_t (& distances)[RAY_SIZE]) const;

    void initialize_packets(const float32_t rpm);

    // make sure packet sizes are correct
    static_assert(sizeof(DataChannel) == 3U, "Error VLP16 data channel size is incorrect");
    static_assert(sizeof(DataBlock) == 100U, "Error VLP16 data block size is incorrect");
    static_assert(sizeof(Packet) == 1206U, "Error VLP16 packet size is incorrect");

    UdpSender<Packet> m_udp_sender;
    std::chrono::steady_clock::time_point m_last_send_time;
    Packet m_all_flat_ground_packet;
    Packet m_flat_ground_wall_packet;
    const std::atomic_bool & m_running;
    const std::chrono::nanoseconds m_send_period;
    uint16_t m_azimuth_increment;
    uint32_t m_send_count = 0;
    std::thread m_thread;
  };  // SpoofTask

  std::atomic_bool m_running;
  SpoofTask m_spoofer;
};  // Vlp16IntegrationSpoofer

}  // namespace lidar_integration
#endif  // LIDAR_INTEGRATION__VLP16_INTEGRATION_SPOOFER_HPP_
