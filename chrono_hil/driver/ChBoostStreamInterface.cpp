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

int ChBoostStreamInterface::Synchronize(int port_in) {

  boost::asio::io_context ioContext;
  udp::socket socket(ioContext,
                     udp::endpoint(udp::v4(), port_in)); // designated port

  float udp_float_arr[3];

  udp::endpoint ep_sender;
  // socket.receive_from(boost::asio::buffer(&udp_float_arr, sizeof(float) *
  // 3),
  //                     ep_sender);

  while (socket.receive_from(
      boost::asio::buffer(&udp_float_arr, sizeof(float) * 3), ep_sender)) {
    m_throttle = udp_float_arr[0];
    m_braking = udp_float_arr[1];
    m_steering = udp_float_arr[2];
    return 1;
  }

  return 0;
}

void ChBoostStreamInterface::StreamDashboard(std::string ip_addr_out,
                                             int port_out, float veh_speed_ms,
                                             float veh_rpm) {

  boost::asio::io_service io_service;
  udp::socket socket(io_service);
  udp::endpoint remote_endpoint =
      udp::endpoint(address::from_string(ip_addr_out), port_out);
  socket.open(udp::v4());

  float udp_veh_dash[2];
  udp_veh_dash[0] = veh_speed_ms;
  udp_veh_dash[1] = veh_rpm;

  boost::system::error_code err;
  auto sent =
      socket.send_to(boost::asio::buffer(udp_veh_dash, sizeof(float) * 2),
                     remote_endpoint, 0, err);

  socket.close();
}

float ChBoostStreamInterface::GetThrottle() { return m_throttle; };

float ChBoostStreamInterface::GetBraking() { return m_braking; };

float ChBoostStreamInterface::GetSteering() { return m_steering; };

} // namespace hil
} // namespace chrono