// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// =============================================================================

#include "ChBoostStreamInterface.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace chrono {
namespace hil {

ChBoostStreamInterface::ChBoostStreamInterface() {}

ChBoostStreamInterface::~ChBoostStreamInterface() {}

void ChBoostStreamInterface::Initialize() {}

int ChBoostStreamInterface::Synchronize(std::string ip_addr_in, int port_in) {
  boost::asio::io_service io_service;
  udp::socket socket(io_service);
  udp::endpoint remote_endpoint =
      udp::endpoint(address::from_string(ip_addr_in), port_in);
  socket.open(udp::v4());
  boost::asio::ip::udp::endpoint sender;
  std::size_t bytes_transferred =
      socket.receive_from(boost::asio::buffer(recv_buffer), sender);
  m_throttle = recv_buffer[0];
  m_braking = recv_buffer[1];
  m_steering = recv_buffer[2];
}

float ChBoostStreamInterface::GetThrottle() { return m_throttle; };

float ChBoostStreamInterface::GetBraking() { return m_braking; };

float ChBoostStreamInterface::GetSteering() { return m_steering; };

} // namespace hil
} // namespace chrono