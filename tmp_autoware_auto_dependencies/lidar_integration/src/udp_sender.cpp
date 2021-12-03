// Copyright 2020 Silexica GmbH, Lichtstr. 25, Cologne, Germany. All rights reserved.
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

#include <lidar_integration/udp_sender.hpp>
#include <common/types.hpp>

#include <string.h>
#include <arpa/inet.h>

#include <stdexcept>
#include <sstream>

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

UdpSenderBase::UdpSenderBase(
  const char8_t * const ip, const uint16_t port,
  const bool8_t ipv6)
: m_socket(-1)
{
  // Create socket
  m_socket = socket(ipv6 ? AF_INET6 : AF_INET, SOCK_DGRAM, 0);
  if (m_socket < 0) {
    throw std::runtime_error("Failed to create socket");
  }

  // Setup address information for connecting
  memset(&m_addr, 0, sizeof(m_addr));
  if (ipv6) {
    m_addr.sin6_family = AF_INET6;
    m_addr.sin6_port = htons(port);
    inet_pton(AF_INET6, ip, &m_addr.sin6_addr.s6_addr);
  } else {
    sockaddr_in * addr = reinterpret_cast<sockaddr_in *>(&m_addr);
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    inet_pton(AF_INET, ip, &addr->sin_addr);
  }

  // Connect to address
  int ret = connect(
    m_socket, reinterpret_cast<sockaddr *>(&m_addr),
    static_cast<socklen_t>(ipv6 ? sizeof(sockaddr_in6) : sizeof(sockaddr_in)));
  if (ret != 0) {
    throw std::runtime_error("Could not connect");
  }
}

void UdpSenderBase::send(const void * const data, const size_t length) const
{
  ::send(m_socket, data, length, 0);
}
