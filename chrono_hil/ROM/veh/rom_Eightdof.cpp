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
// Authors: Huzaifa Mustafa Unjhawala, Jason Zhou
// =============================================================================
//
// The 8dof vehicle model base class
// The class includes 8dof vehicle dynamics computation
//
// =============================================================================

#include "rom_Eightdof.h"

using namespace chrono;
using namespace chrono::vehicle;

/*
Code for the Eight dof model implemented in cpp
*/

// sets the vertical forces based on the vehicle weight
void vehInit(VehicleState &v_state, const VehicleParam &v_param) {
  double weight_split =
      ((v_param._m * G * v_param._b) / (2 * (v_param._a + v_param._b)) +
       v_param._muf * G);
  v_state._fzlf = v_state._fzrf = weight_split;

  weight_split =
      ((v_param._m * G * v_param._b) / (2 * (v_param._a + v_param._b)) +
       v_param._mur * G);

  v_state._fzlr = v_state._fzrr = weight_split;
}

// returns drive toruqe at a given omega
double driveTorque(const VehicleParam &v_params, const double throttle,
                   const double omega) {

  double motor_speed = omega / v_params._gearRatio;
  double motor_torque =
      v_params._maxTorque -
      (motor_speed * (v_params._maxTorque / v_params._maxSpeed));

  motor_torque =
      motor_torque * throttle - v_params._c1 * motor_speed - v_params._c0;

  if (motor_torque < 0) {
    motor_torque = 0;
  }
  return motor_torque / v_params._gearRatio;
}

/*
function to advance the time step of the 8DOF vehicle
along with the vehicle state that will be updated, we pass the
vehicle paramters , a vector containing the longitudinal forces,
and a vector containing the lateral forces by reference, the front
and rear unspring mass positions (loaded tire radius) and the controls
--- There is no point passing all the tire states and paramters
since none of the are used here - however, there is thus an additional
copy in main for getting these states into a vector. So thus, do not know
if this way is actually faster, but I think it is much cleaner than passing
4 tire states and paramaters
*/

void vehAdv(VehicleState &v_states, const VehicleParam &v_params,
            const std::vector<double> &fx, const std::vector<double> &fy,
            const double huf, const double hur) {

  // get the total mass of the vehicle and the vertical distance from the sprung
  // mass C.M. to the vehicle
  double mt = v_params._m + 2 * (v_params._muf + v_params._mur);
  double hrc = (v_params._hrcf * v_params._b + v_params._hrcr * v_params._a) /
               (v_params._a + v_params._b);

  // a bunch of varaibles to simplify the formula
  double E1 =
      -mt * v_states._wz * v_states._u + (fy[0] + fy[1] + fy[2] + fy[3]);

  double E2 = (fy[0] + fy[1]) * v_params._a - (fy[2] + fy[3]) * v_params._b +
              (fx[1] - fx[0]) * v_params._cf / 2 +
              (fx[3] - fx[2]) * v_params._cr / 2 +
              (-v_params._muf * v_params._a + v_params._mur * v_params._b) *
                  v_states._wz * v_states._u;

  double E3 = v_params._m * G * hrc * v_states._phi -
              (v_params._krof + v_params._kror) * v_states._phi -
              (v_params._brof + v_params._bror) * v_states._wx +
              hrc * v_params._m * v_states._wz * v_states._u;

  double A1 = v_params._mur * v_params._b - v_params._muf * v_params._a;

  double A2 = v_params._jx + v_params._m * std::pow(hrc, 2);

  double A3 = hrc * v_params._m;

  // Integration using half implicit - level 2 variables found first in next
  // time step

  // update the acceleration states - level 2 variables

  v_states._udot =
      v_states._wz * v_states._v +
      (1 / mt) * ((fx[0] + fx[1] + fx[2] + fx[3]) +
                  (-v_params._mur * v_params._b + v_params._muf * v_params._a) *
                      std::pow(v_states._wz, 2) -
                  2. * hrc * v_params._m * v_states._wz * v_states._wx);

  // common denominator
  double denom = (A2 * std::pow(A1, 2) - 2. * A1 * A3 * v_params._jxz +
                  v_params._jz * std::pow(A3, 2) +
                  mt * std::pow(v_params._jxz, 2) - A2 * v_params._jz * mt);

  v_states._vdot = (E1 * std::pow(v_params._jxz, 2) - A1 * A2 * E2 +
                    A1 * E3 * v_params._jxz + A3 * E2 * v_params._jxz -
                    A2 * E1 * v_params._jz - A3 * E3 * v_params._jz) /
                   denom;

  v_states._wxdot = (std::pow(A1, 2) * E3 - A1 * A3 * E2 +
                     A1 * E1 * v_params._jxz - A3 * E1 * v_params._jz +
                     E2 * v_params._jxz * mt - E3 * v_params._jz * mt) /
                    denom;

  v_states._wzdot =
      (std::pow(A3, 2) * E2 - A1 * A2 * E1 - A1 * A3 * E3 +
       A3 * E1 * v_params._jxz - A2 * E2 * mt + E3 * v_params._jxz * mt) /
      denom;

  // update the level 1 varaibles using the next time step level 2 variable
  v_states._u = v_states._u + v_params._step * v_states._udot;
  v_states._v = v_states._v + v_params._step * v_states._vdot;
  v_states._wx = v_states._wx + v_params._step * v_states._wxdot;
  v_states._wz = v_states._wz + v_params._step * v_states._wzdot;

  // update the level 0 varaibles using the next time step level 1 varibales
  // over here still using the old psi and phi.. should we update psi and phi
  // first and then use those????

  v_states._x =
      v_states._x + v_params._step * (v_states._u * std::cos(v_states._psi) -
                                      v_states._v * std::sin(v_states._psi));

  v_states._y =
      v_states._y + v_params._step * (v_states._u * std::sin(v_states._psi) +
                                      v_states._v * std::cos(v_states._psi));

  v_states._psi = v_states._psi + v_params._step * v_states._wz;
  v_states._phi = v_states._phi + v_params._step * v_states._wx;

  // update the vertical forces
  // sketchy load transfer technique

  double Z1 =
      (v_params._m * G * v_params._b) / (2. * (v_params._a + v_params._b)) +
      (v_params._muf * G) / 2.;

  double Z2 = ((v_params._muf * huf) / v_params._cf +
               v_params._m * v_params._b * (v_params._h - v_params._hrcf) /
                   (v_params._cf * (v_params._a + v_params._b))) *
              (v_states._vdot + v_states._wz * v_states._u);

  double Z3 = (v_params._krof * v_states._phi + v_params._brof * v_states._wx) /
              v_params._cf;

  double Z4 =
      ((v_params._m * v_params._h + v_params._muf * huf + v_params._mur * hur) *
       (v_states._udot - v_states._wz * v_states._v)) /
      (2. * (v_params._a + v_params._b));

  // evaluate the vertical forces for front
  v_states._fzlf = (Z1 - Z2 - Z3 - Z4) > 0. ? (Z1 - Z2 - Z3 - Z4) : 0.;
  v_states._fzrf = (Z1 + Z2 + Z3 - Z4) > 0. ? (Z1 + Z2 + Z3 - Z4) : 0.;

  Z1 = (v_params._m * G * v_params._a) / (2. * (v_params._a + v_params._b)) +
       (v_params._mur * G) / 2.;

  Z2 = ((v_params._mur * hur) / v_params._cr +
        v_params._m * v_params._a * (v_params._h - v_params._hrcr) /
            (v_params._cr * (v_params._a + v_params._b))) *
       (v_states._vdot + v_states._wz * v_states._u);

  Z3 = (v_params._kror * v_states._phi + v_params._bror * v_states._wx) /
       v_params._cr;

  // evaluate vertical forces for the rear
  v_states._fzlr = (Z1 - Z2 - Z3 + Z4) > 0. ? (Z1 - Z2 - Z3 + Z4) : 0.;
  v_states._fzrr = (Z1 + Z2 + Z3 + Z4) > 0. ? (Z1 + Z2 + Z3 + Z4) : 0.;
}

void vehToTireTransform(TMeasyState &tirelf_st, TMeasyState &tirerf_st,
                        TMeasyState &tirelr_st, TMeasyState &tirerr_st,
                        const VehicleState &v_states,
                        const VehicleParam &v_params,
                        const std::vector<double> &controls) {

  // get the controls and time out
  double t = controls[0];
  double delta = controls[1] * v_params._maxSteer;
  double throttle = controls[2];
  double brake = controls[3];

  // left front
  tirelf_st._fz = v_states._fzlf;
  tirelf_st._vsy = v_states._v + v_states._wz * v_params._a;
  tirelf_st._vsx =
      (v_states._u - (v_states._wz * v_params._cf) / 2.) * std::cos(delta) +
      tirelf_st._vsy * std::sin(delta);

  // right front
  tirerf_st._fz = v_states._fzrf;
  tirerf_st._vsy = v_states._v + v_states._wz * v_params._a;
  tirerf_st._vsx =
      (v_states._u + (v_states._wz * v_params._cf) / 2.) * std::cos(delta) +
      tirerf_st._vsy * std::sin(delta);

  // left rear - No steer
  tirelr_st._fz = v_states._fzlr;
  tirelr_st._vsy = v_states._v - v_states._wz * v_params._b;
  tirelr_st._vsx = v_states._u - (v_states._wz * v_params._cr) / 2.;

  // rigth rear - No steer
  tirerr_st._fz = v_states._fzrr;
  tirerr_st._vsy = v_states._v - v_states._wz * v_params._b;
  tirerr_st._vsx = v_states._u + (v_states._wz * v_params._cr) / 2.;
}

void tireToVehTransform(TMeasyState &tirelf_st, TMeasyState &tirerf_st,
                        TMeasyState &tirelr_st, TMeasyState &tirerr_st,
                        const VehicleState &v_states,
                        const VehicleParam &v_params,
                        const std::vector<double> &controls) {

  // get the controls and time out
  double t = controls[0];
  double delta = controls[1] * v_params._maxSteer;
  double throttle = controls[2];
  double brake = controls[3];

  double _fx, _fy;

  // left front
  _fx = tirelf_st._fx * std::cos(delta) - tirelf_st._fy * std::sin(delta);
  _fy = tirelf_st._fx * std::sin(delta) + tirelf_st._fy * std::cos(delta);
  tirelf_st._fx = _fx;
  tirelf_st._fy = _fy;

  // right front
  _fx = tirerf_st._fx * std::cos(delta) - tirerf_st._fy * std::sin(delta);
  _fy = tirerf_st._fx * std::sin(delta) + tirerf_st._fy * std::cos(delta);
  tirerf_st._fx = _fx;
  tirerf_st._fy = _fy;

  // rear tires - No steer so no need to transform
}

// setting Vehicle parameters using a JSON file
void setVehParamsJSON(VehicleParam &v_params, std::string fileName) {

  // parse the stream into DOM tree
  rapidjson::Document d;
  vehicle::ReadFileJSON(fileName, d);

  if (d.HasParseError()) {
    std::cout << "Error with rapidjson:" << std::endl
              << d.GetParseError() << std::endl;
  }

  // the file should have all these parameters defined
  v_params._a = d["a"].GetDouble();
  v_params._b = d["b"].GetDouble();
  v_params._m = d["m"].GetDouble();
  v_params._h = d["h"].GetDouble();
  v_params._jz = d["jz"].GetDouble();
  v_params._jx = d["jx"].GetDouble();
  v_params._jxz = d["jxz"].GetDouble();
  v_params._cf = d["cf"].GetDouble();
  v_params._cr = d["cr"].GetDouble();
  v_params._muf = d["muf"].GetDouble();
  v_params._mur = d["mur"].GetDouble();
  v_params._hrcf = d["hrcf"].GetDouble();
  v_params._hrcr = d["hrcr"].GetDouble();
  v_params._krof = d["krof"].GetDouble();
  v_params._kror = d["kror"].GetDouble();
  v_params._brof = d["brof"].GetDouble();
  v_params._bror = d["bror"].GetDouble();
  v_params._maxSteer = d["maxSteer"].GetDouble();
  v_params._gearRatio = d["gearRatio"].GetDouble();
  v_params._maxTorque = d["maxTorque"].GetDouble();
  v_params._maxBrakeTorque = d["maxBrakeTorque"].GetDouble();
  v_params._maxSpeed = d["maxSpeed"].GetDouble();
  v_params._c1 = d["c1"].GetDouble();
  v_params._c0 = d["c0"].GetDouble();
  v_params._step = d["step"].GetDouble();
}