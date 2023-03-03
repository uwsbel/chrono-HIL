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

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/driver/ChSDLInterface.h"

#include "chrono_hil/network/ChBoostInStreamer.h"
#include "chrono_hil/network/ChBoostOutStreamer.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace chrono;
using namespace chrono::hil;

using boost::asio::ip::address;
using boost::asio::ip::udp;

#define PORT_IN 1204
#define PORT_OUT 1209
#define IP_OUT "127.0.0.1"

int main(int argc, char *argv[]) {
  ChSDLInterface SDLDriver;

  SDLDriver.Initialize();

  SDLDriver.SetJoystickConfigFile(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                  std::string("/joystick/controller_G29.json"));

  ChBoostOutStreamer out_streamer(IP_OUT, PORT_OUT);
  ChBoostInStreamer in_streamer(PORT_IN, 13);

  std::vector<float> recv_data;

  while (true) {
    float throttle = SDLDriver.GetThrottle();
    float steering = SDLDriver.GetSteering();
    float braking = SDLDriver.GetBraking();
    out_streamer.AddData(throttle);
    out_streamer.AddData(steering);
    out_streamer.AddData(braking);
    out_streamer.Synchronize();

    in_streamer.Synchronize();
    recv_data = in_streamer.GetRecvData();
    for (int i = 0; i < recv_data.size(); i++) {
      std::cout << recv_data[i] << ",";
    }
    std::cout << std::endl;

    if (SDLDriver.Synchronize() == 1) {
      break;
    }
  }

  return 0;
}
