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
// Authors: Simone Benatti
// =============================================================================
//
// Custom drivers for the NSF project.
// Both are specialization of ChPathFollowerDriver
// The leader will adjust its behavior depending on the traveled distance
// The follower will adjust the speed to reach a target gap with the leader
//
// =============================================================================

#ifndef CH_2DOF_H
#define CH_2DOF_H

#include <string>

#include "../ChApiHil.h"

namespace chrono {
namespace hil {

class CH_HIL_API SimplifiedVehicle_2DOF {
public:
  /// Construct an interactive driver.
  SimplifiedVehicle_2DOF() {}

  ~SimplifiedVehicle_2DOF() {}

private:
  // vehicle properties variables
  float a = 1.14;       // distance of c.g. from front axle (m)
  float b = 1.4;        // distance of c.g. from rear axle  (m)
  float h = 0.713;      // height of c.g. from the ground (m)
  float Cf = 88000.0;   // front axle cornering stiffness (N/rad)
  float Cr = 88000.0;   // rear axle cornering stiffness (N/rad)
  float Cxf = 10000.0;  // front axle longitudinal stiffness (N)
  float Cxr = 10000.0;  // rear axle longitudinal stiffness (N)
  float ktf = 326332.0; // tire stiffness - front (N/m)
  float ktr = 326332.0; // tire stiffmess - rear (N/m)
  float m = 1720.0;     // the mass of the vehicle (kg)
  float muf = 120.0;    // the front unsprung mass (kg)
  float mur = 120.0;    // the rear unsprung mass (kg)
  float Jz = 2420.0;    // yaw moment of inertia (kg.m^2)
  float r0 = 0.285;     // wheel radius
  float Jw = 2.0;       // wheel roll inertia
  float rr = 0.0125;    // Rolling resistance

  // vehicle state variables
  float x = 0.0;        // x coordinate
  float y = 0.0;        // y coordinate
  float Vx = 0.0;       // lateral velocity
  float Vy = 0.0;       // longitudinal velocity
  float yaw = 0.0;      // yaw rate
  float yaw_rate = 0.0; // yaw rate
  float wf = 0.0;       // front wheel angular velocity
  float wr = 0.0;       // rear wheel angular velocity
};

} // namespace hil
} // namespace chrono

#endif
