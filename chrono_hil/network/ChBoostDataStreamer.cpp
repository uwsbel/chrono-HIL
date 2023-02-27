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
// This is a stream-based input driver interface based on boost UDP networking
// =============================================================================

#include "ChBoostDataStreamer.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace chrono {
namespace hil {

ChBoostDataStreamer::ChBoostDataStreamer(std::string end_ip_addr, int port) {
  this->end_ip_addr = end_ip_addr;
  this->port = port;
}

void ChBoostDataStreamer::AddData(float data_in) {
  stream_data.push_back(data_in);
}

void ChBoostDataStreamer::Synchronize() {
  boost::asio::io_service io_service;
  udp::socket socket(io_service);
  udp::endpoint remote_endpoint =
      udp::endpoint(address::from_string(end_ip_addr), port);
  socket.open(udp::v4());

  boost::system::error_code err;
  auto sent =
      socket.send_to(boost::asio::buffer(stream_data.data(),
                                         sizeof(float) * stream_data.size()),
                     remote_endpoint, 0, err);

  socket.close();
  stream_data.clear();
}

} // namespace hil
} // namespace chrono