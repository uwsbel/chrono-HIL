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
// 2DOF vehicle model class
//
// =============================================================================
#include "SimplifiedVehicle_2DOF.h"
#include <cmath>
#include <iostream>
#include <string>

namespace chrono {
namespace hil {

void SimplifiedVehicle_2DOF::Initialize() {
  //  Vertical forces initially
  Fzgf = (m * g * b) / (a + b) + muf * g;
  Fzgr = (m * g * a) / (a + b) + mur * g;

  // Tire compression xt initially
  xtf = Fzgf / ktf;
  xtr = Fzgr / ktr;
}

void SimplifiedVehicle_2DOF::Step(float throt, float brake, float steer,
                                  float time_step) {
  // read inputs
  float steer_ang = steer * max_steer;
  float throt_torque = throt * max_torque;
  float brake_torque = brake * max_brake;

  // total vehicle mass
  float mt = m + mur + muf;

  // tire properties
  float Rf = r0 - xtf;
  float Rr = r0 - xtr;

  // slip ratio
  float sf =
      ((Rf * wf) - Vx * cos(steer_ang) + (Vy + a * yaw_rate) * sin(steer_ang)) /
      (Vx * cos(steer_ang) + (Vy + a * yaw_rate) * sin(steer_ang));

  float sr = (Rr * wr - Vx) / Vx;

  std::cout << "sf:" << sf << "sr:" << sr << std::endl;

  // ODE 1/5
  float dVy = (-Vx * yaw_rate) +
              (Cf * (Vy + a * yaw_rate) / Vx - steer_ang) / mt +
              (Cr * (Vy - b * yaw_rate) / Vx) / mt;
  // ODE 2/5
  float dVx = (Vy * yaw_rate) + (sf * Cxf + sr * Cxr) / mt -
              Cf * ((Vy + a * yaw_rate) / Vx - steer_ang) * steer_ang;
  // ODE 3/5
  float d_yaw_rate =
      (1 / Jz) * ((a * Cf * ((Vy + a * yaw_rate) / Vx - steer_ang)) -
                  b * Cr * ((Vy - b * yaw_rate) / Vx));
  // ODE 4/5
  float dy = Vx * sin(yaw) + Vy * cos(yaw);
  // ODE 5/5
  float dx = Vx * cos(yaw) - Vy * sin(yaw);

  // integration
  y = y + dy * time_step;
  x = x + dx * time_step;
  yaw_rate = yaw_rate + d_yaw_rate * time_step;
  yaw = yaw + yaw_rate * time_step;
  Vx = Vx + dVx * time_step;
  Vy = Vy + dVy * time_step;

  // update inputs
  float dwf = throt_torque / Jw - brake_torque / Jw;
  float dwr = throt_torque / Jw - brake_torque / Jw;
  wf = wf + dwf * time_step;
  wr = wr + dwr * time_step;
  if (wf < 0) {
    wf = 0;
  }
  if (wr < 0) {
    wr = 0;
  }
}

} // namespace hil
} // namespace chrono
