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
// Authors: Simone Benatti, Jason Zhou
// =============================================================================
//
// Interactive driver for a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use that as an input; it will
// otherwise default to a keyboard input.
//
// =============================================================================

#include "ChCSLSoundEngine.h"
#include <string>
#ifdef CHRONO_IRRKLANG
namespace chrono {
namespace hil {

ChCSLSoundEngine::ChCSLSoundEngine(ChVehicle *vehicle) {
  thisvehicle = vehicle;
  sound_engine = irrklang::createIrrKlangDevice();
  car_sound = sound_engine->play2D((std::string(STRINGIFY(HIGHWAY_DATA_DIR)) +
                                    "/Environments/Iowa/Sounds/audi_dr.ogg")
                                       .c_str(),
                                   true, false, true);
  car_sound->setIsPaused(true);
}

ChCSLSoundEngine::~ChCSLSoundEngine() { delete sound_engine; }

void ChCSLSoundEngine::Synchronize(double time) {
  // update every 0.01 sec
  if (time - last_time_played > 0.2) {
    last_time_played = time;
    int cur_gear = thisvehicle->GetPowertrain()->GetCurrentTransmissionGear();
    double rpm = thisvehicle->GetPowertrain()->GetMotorSpeed() * 60 / CH_C_2PI;
    if (cur_gear == 1) {
      double soundspeed = rpm / (10000.); // denominator: to guess
      if (soundspeed < 0.1)
        soundspeed = 0.1;
      if (car_sound->getIsPaused())
        car_sound->setIsPaused(false);
      car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
    } else if (cur_gear == 2) {
      double soundspeed = rpm / (8000.); // denominator: to guess
      if (soundspeed < 0.1)
        soundspeed = 0.1;
      if (car_sound->getIsPaused())
        car_sound->setIsPaused(false);
      car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
    } else if (cur_gear == 3) {
      double soundspeed = rpm / (8000.); // denominator: to guess
      if (soundspeed < 0.1)
        soundspeed = 0.1;
      if (car_sound->getIsPaused())
        car_sound->setIsPaused(false);
      car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
    } else {
      double soundspeed = rpm / (6000.); // denominator: to guess
      if (soundspeed < 0.1)
        soundspeed = 0.1;
      if (car_sound->getIsPaused())
        car_sound->setIsPaused(false);
      car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
    }
  }
}
} // namespace hil
} // namespace chrono

#endif
