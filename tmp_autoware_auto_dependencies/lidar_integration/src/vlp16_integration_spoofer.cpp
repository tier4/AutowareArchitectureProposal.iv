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

#include <common/types.hpp>

#include "lidar_integration/vlp16_integration_spoofer.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::float32_t;

namespace lidar_integration
{

Vlp16IntegrationSpoofer::Vlp16IntegrationSpoofer(
  const char8_t * const ip,
  const uint16_t port,
  const float32_t rpm)
: m_running(false),
  m_spoofer(ip, port, rpm, m_running)
{
}

Vlp16IntegrationSpoofer::~Vlp16IntegrationSpoofer()
{
  stop();
}

void Vlp16IntegrationSpoofer::start()
{
  m_running.store(true);
  m_spoofer.start();
}

void Vlp16IntegrationSpoofer::stop()
{
  m_running.store(false);
  m_spoofer.stop();
}

/// rpm min speed
static constexpr float32_t MIN_RPM = 300.0F;
/// rpm max speed
static constexpr float32_t MAX_RPM = 1200.0F;

Vlp16IntegrationSpoofer::SpoofTask::SpoofTask(
  const char8_t * const ip,
  const uint16_t port,
  const float32_t rpm,
  const std::atomic<bool8_t> & running)
: m_udp_sender(ip, port),
  m_last_send_time(std::chrono::nanoseconds(0)),
  m_all_flat_ground_packet({}),
  m_flat_ground_wall_packet({}),
  m_running(running),
  m_send_period(std::chrono::microseconds(1000000LL / 754LL)),
  // 754 packets/second, from spec sheet
  m_azimuth_increment(0)
{
  initialize_packets(rpm);
  const float32_t period_ms = 60.0E3F / rpm;
  const float32_t dth_tic =
    55.296E-3F * 2.0F * NUM_BLOCKS_PER_PACKET *
    static_cast<float32_t>(AZIMUTH_ROTATION_RESOLUTION) / period_ms;
  m_azimuth_increment = static_cast<uint16_t>(dth_tic);
}

void Vlp16IntegrationSpoofer::SpoofTask::task_function()
{
  using namespace std::chrono_literals;   // NOLINT
  using std::chrono::steady_clock;

  steady_clock::time_point last_send_time(steady_clock::now());
  bool8_t just_sent_all_ground = true;
  update_packet_azimuth(m_all_flat_ground_packet, m_azimuth_increment);
  while (m_running.load(std::memory_order_relaxed)) {
    const steady_clock::time_point right_now(steady_clock::now());
    if ((right_now - last_send_time) > m_send_period) {
      if (just_sent_all_ground) {
        // update azimuth
        update_packet_azimuth(
          m_flat_ground_wall_packet,
          static_cast<uint16_t>(2U * m_azimuth_increment));
        m_udp_sender.send(m_flat_ground_wall_packet);
      } else {
        // update azimuth
        update_packet_azimuth(
          m_all_flat_ground_packet,
          static_cast<uint16_t>(2U * m_azimuth_increment));
        m_udp_sender.send(m_all_flat_ground_packet);
      }
      just_sent_all_ground = !just_sent_all_ground;
      last_send_time = right_now;
      m_send_count += 1;
    }

    std::this_thread::sleep_for(100us);
  }
}

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

void Vlp16IntegrationSpoofer::SpoofTask::uint16_to_bytes(const uint16_t val, uint8_t arr[])
{
  arr[0U] = static_cast<uint8_t>(val & 0xFFU);
  arr[1U] = static_cast<uint8_t>(val >> 8U);
}

uint16_t Vlp16IntegrationSpoofer::SpoofTask::bytes_to_uint16(const uint8_t arr[])
{
  return static_cast<uint16_t>((arr[1U] << 8U) + arr[0U]);
}

void Vlp16IntegrationSpoofer::SpoofTask::update_packet_azimuth(
  Packet & pkt, const uint16_t dth_tic)
{
  for (uint32_t idx = 0U; idx < NUM_BLOCKS_PER_PACKET; ++idx) {
    DataBlock & blk = pkt.blocks[idx];
    // azimuth
    uint16_t th_tic = bytes_to_uint16(blk.azimuth_bytes);
    th_tic = static_cast<uint16_t>((th_tic + dth_tic) % AZIMUTH_ROTATION_RESOLUTION);
    uint16_to_bytes(th_tic, blk.azimuth_bytes);
  }
}

void Vlp16IntegrationSpoofer::SpoofTask::init_packet(
  Packet & pkt,
  const float32_t rpm,
  const uint16_t (& distances)[RAY_SIZE]) const
{
  // timestamp is not read atm
  // factory bytes also not read
  // delta azimuth per index based on rpm
  const float32_t period_ms = 60.0E3F / rpm;
  const float32_t dth_tic =
    2.0F * 55.296E-3F * static_cast<float32_t>(AZIMUTH_ROTATION_RESOLUTION) / period_ms;
  for (uint32_t idx = 0U; idx < NUM_BLOCKS_PER_PACKET; ++idx) {
    DataBlock & blk = pkt.blocks[idx];
    // flag
    blk.flag[0U] = static_cast<uint8_t>(0xFF);
    blk.flag[1U] = static_cast<uint8_t>(0xEE);
    // azimuth
    uint16_to_bytes(
      static_cast<uint16_t>(dth_tic * static_cast<float32_t>(idx)),
      blk.azimuth_bytes);
    for (uint32_t jdx = 0U; jdx < NUM_POINTS_PER_BLOCK; ++jdx) {
      DataChannel & data = blk.channels[jdx];
      uint16_to_bytes(distances[jdx % RAY_SIZE], data.data);
      // intensity
      data.data[2U] = static_cast<uint8_t>(100U);
    }
  }
}

void Vlp16IntegrationSpoofer::SpoofTask::initialize_packets(const float32_t rpm)
{
  const float32_t all_flat_ground_distances_m[RAY_SIZE] = {
    10.3082077832F,
    70.0047387525F,
    11.8790393175F,
    0.0F,  // 70.0426680288F,
    14.0236477175F,
    0.0F,  // 70.1186294229F,
    17.1245367095F,
    0.0F,  // 70.232829299F,
    22.0013602226F,
    0.0F,  // 70.3855789151F,
    30.7852227581F,
    0.0F,  // horizon: > 100m
    51.290181248F,
    0.0F,  // horizon: > 100m
    0.0F,  // 70.0047387525F,
    0.0F   // horizon: > 100m
  };
  uint16_t all_flat_ground_distance_tic[RAY_SIZE];
  for (uint32_t idx = 0U; idx < RAY_SIZE; ++idx) {
    all_flat_ground_distance_tic[idx] =
      static_cast<uint16_t>(all_flat_ground_distances_m[idx] * 500.0F);
  }
  init_packet(m_all_flat_ground_packet, rpm, all_flat_ground_distance_tic);
  const float32_t flat_ground_wall_distances_m[RAY_SIZE] = {
    10.3082077832F,
    20.0013539293F,
    11.8790393175F,
    20.0121908654F,
    14.0236477175F,
    20.0338941208F,
    17.1245367095F,
    20.0665226568F,
    20.0665226568F,
    20.1101654043F,
    20.0338941208F,
    20.1649418573F,
    20.0121908654F,
    20.2310028762F,
    20.0013539293F,
    20.3085317098F
  };
  uint16_t flat_ground_wall_distance_tic[RAY_SIZE];
  for (uint32_t idx = 0U; idx < RAY_SIZE; ++idx) {
    flat_ground_wall_distance_tic[idx] =
      static_cast<uint16_t>(flat_ground_wall_distances_m[idx] * 500.0F);
  }
  init_packet(m_flat_ground_wall_packet, rpm, flat_ground_wall_distance_tic);
}
}  // namespace lidar_integration
