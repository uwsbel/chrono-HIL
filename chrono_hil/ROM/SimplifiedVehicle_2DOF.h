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
// 2DOF vehicle model class header file
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

  void Initialize();

  void Step(float throt, float brake, float steer, float time_step);

  float GetXPos() { return x; }
  float GetYPos() { return y; }
  float GetYaw() { return yaw; }

private:
  float g = 9.81; // gravitational pull

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
  float x = 0.0;        // x coordinate (expressed in global frame)
  float y = 0.0;        // y coordinate (expressed in global frame)
  float Vx = 3.0;       // longitudinal velocity (expressed in vehicle frame)
  float Vy = 0.0;       // lateral velocity (expressed in vehicle frame)
  float yaw = 0.0;      // yaw rate
  float yaw_rate = 0.0; // yaw rate
  float wf = Vx / r0;   // front wheel angular velocity
  float wr = Vx / r0;   // rear wheel angular velocity

  float Fzgf = 0.0; // weight on the front wheel (we assume an underlying
                    // bicycle model with 2 wheels)
  float Fzgr = 0.0; // weight on the rear wheel (we assume an underlying bicycle
                    // model with 2 wheels)
  float xtf = 0.0;  // front tire compression
  float xtr = 0.0;  // rear tire compression

  // vehicle control variables
  float max_steer = 0.6525249; // vehicle steering angle, in rad
  float max_torque = 300;
  float max_brake = 300;
};

} // namespace hil
} // namespace chrono

#endif
