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
// This is a class which intends to replace ChIrrGuiDriver with SDL backed
// joystick input reading
// This class is not currently inheriting ChDriver, as it's also intended to
// support ROM vehicle model control
//
// =============================================================================

#ifndef CH_SDL_INTERFACE_H
#define CH_SDL_INTERFACE_H

#include <string>

#include "../ChApiHil.h"
#include <SDL2/SDL.h>

struct SDLAxis {
  int axis;
  int max;
  int min;
  int scaled_max;
  int scaled_min;
};

struct {
  int button;
} SDLButton; // Structure variable

namespace chrono {
namespace hil {

// Driver for the leader vehicle, it adjusts its target speed according to a
// piecewise sinusoidal function In the buffer-areas between pieces it keeps the
// target speed specified in target_speed
class CH_HIL_API ChSDLInterface {
public:
  /// Construct an interactive driver.
  ChSDLInterface();

  ~ChSDLInterface();

  void Initialize();

  void SetJoystickConfigFile(std::string config_filename);

  float GetThrottle();

  float GetBraking();

  float GetSteering();

  int Synchronize();

private:
  SDL_Joystick *m_joystick;

  SDLAxis m_throttle_axis;
  SDLAxis m_braking_axis;
  SDLAxis m_steering_axis;
};

} // namespace hil
} // namespace chrono
#endif