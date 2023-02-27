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
// Authors: Jason Zhou
// =============================================================================
// This is a stream-based input driver interface based on boost UDP networking
// =============================================================================

#ifndef CH_BOOST_DATASTREAMER_H
#define CH_BOOST_DATASTREAMER_H

#include <string>

#include "../ChApiHil.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::address;
using boost::asio::ip::udp;

namespace chrono {
namespace hil {

// Driver for the leader vehicle, it adjusts its target speed according to a
// piecewise sinusoidal function In the buffer-areas between pieces it keeps the
// target speed specified in target_speed
class CH_HIL_API ChBoostDataStreamer {
public:
  /// Construct an interactive driver.
  ChBoostDataStreamer(std::string end_ip_addr, int port);

  ~ChBoostDataStreamer(){};

  void AddData(float data_in);

  void Synchronize();

private:
  std::vector<float> stream_data;
  std::string end_ip_addr;
  int port;
};

} // namespace hil
} // namespace chrono
#endif