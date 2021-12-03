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


#ifndef LIDAR_INTEGRATION__UDP_SENDER_HPP_
#define LIDAR_INTEGRATION__UDP_SENDER_HPP_

#include <lidar_integration/visibility_control.hpp>
#include <common/types.hpp>

#include <stdint.h>
#include <netinet/in.h>

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

class LIDAR_INTEGRATION_PUBLIC UdpSenderBase
{
public:
  UdpSenderBase(
    const char8_t * const ip, const uint16_t port,
    const bool8_t ipv6 = false);

protected:
  void send(const void * const data, const size_t length) const;

private:
  int m_socket;
  struct sockaddr_in6 m_addr;
};

template<typename Packet>
class UdpSender : public UdpSenderBase
{
public:
  using UdpSenderBase::UdpSenderBase;

public:
  void send(Packet const & pkt) const
  {
    UdpSenderBase::send(static_cast<const void * const>(&pkt), sizeof(pkt));
  }
};

#endif  // LIDAR_INTEGRATION__UDP_SENDER_HPP_
