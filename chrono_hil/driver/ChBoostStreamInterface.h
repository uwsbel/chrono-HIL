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
//
// =============================================================================

#ifndef CH_BOOST_INTERFACE_H
#define CH_BOOST_INTERFACE_H

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
class CH_HIL_API ChBoostStreamInterface {
public:
  /// Construct an interactive driver.
  ChBoostStreamInterface();

  ~ChBoostStreamInterface();

  void Initialize();

  float GetThrottle();

  float GetBraking();

  float GetSteering();

  int Synchronize(int port_in);

  void StreamDashboard(std::string ip_addr_out, int port_out,
                       float veh_speed_ms, float veh_rpm);

private:
  float m_throttle;
  float m_braking;
  float m_steering;
};

} // namespace hil
} // namespace chrono
#endif