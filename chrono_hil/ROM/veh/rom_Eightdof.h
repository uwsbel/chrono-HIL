#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "rom_TMeasy.h"
#include "rom_utils.h"
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#ifndef EIGHTDOF_H
#define EIGHTDOF_H

/*
Header file for the 8 DOF model implemented in cpp
*/

// forward declaration
struct TMeasyState;
struct TMeasyParam;

// vehicle Parameters structure
struct VehicleParam {

  // default constructor with pre tuned values from HMMVW calibration
  VehicleParam()
      : _a(1.6889), _b(1.6889), _h(0.713), _m(2097.85), _jz(4519.), _jx(1289.),
        _jxz(3.265), _cf(1.82), _cr(1.82), _muf(127.866), _mur(129.98),
        _hrcf(0.379), _hrcr(0.327), _krof(31000), _kror(31000), _brof(3300),
        _bror(3300), _maxSteer(0.6525249), _gearRatio(0.06), _maxTorque(1000.),
        _maxBrakeTorque(4000.), _maxSpeed(500.), _c1(0.), _c0(0.), _step(1e-2) {
  }

  // constructor
  VehicleParam(double a, double b, double h, double m, double Jz, double Jx,
               double Jxz, double cf, double cr, double muf, double mur,
               double hrcf, double hrcr, double krof, double kror, double brof,
               double bror, double maxSteer, double gearRatio, double maxTorque,
               double brakeTorque, double maxSpeed, double c1, double c0,
               double step)
      : _a(a), _b(b), _h(h), _m(m), _jz(Jz), _jx(Jx), _jxz(Jxz), _cf(cf),
        _cr(cr), _muf(muf), _mur(mur), _hrcf(hrcf), _hrcr(hrcr), _krof(krof),
        _kror(kror), _brof(bror), _bror(bror), _maxSteer(maxSteer),
        _gearRatio(gearRatio), _maxTorque(maxTorque),
        _maxBrakeTorque(brakeTorque), _maxSpeed(maxSpeed), _c1(c1), _c0(c0),
        _step(step) {}

  double _a, _b;   // distance c.g. - front axle & distance c.g. - rear axle (m)
  double _h;       // height of c.g
  double _m;       // total vehicle mass (kg)
  double _jz;      // yaw moment inertia (kg.m^2)
  double _jx;      // roll inertia
  double _jxz;     // XZ inertia
  double _cf, _cr; // front and rear track width
  double _muf, _mur;   // front and rear unsprung mass
  double _hrcf, _hrcr; // front and rear roll centre height below C.g
  double _krof, _kror, _brof,
      _bror; // front and rear roll stiffness and damping

  // max steer angle parameters of the vehicle
  double _maxSteer;

  // some throttle parameters of the vehicle
  double _gearRatio;      // gear ratio
  double _maxTorque;      // Max torque
  double _maxBrakeTorque; // max brake torque
  double _maxSpeed;       // Max speed
  double _c1, _c0;        // motor resistance - mainly needed for rc car

  double _step; // vehicle integration time step
};

// vehicle states structure
struct VehicleState {

  // default constructor just assigns zero to all members
  VehicleState()
      : _x(0.), _y(0.), _u(0.), _v(0.), _psi(0.), _wz(0.), _phi(0.), _wx(0.),
        _udot(0.), _vdot(0.), _wxdot(0.), _wzdot(0.), _fzlf(0.), _fzrf(0.),
        _fzlr(0.), _fzrr(0.) {}

  // special constructor in case need to start simulation
  // from some other state
  double _x, _y;    // x and y position
  double _u, _v;    // x and y velocity
  double _psi, _wz; // yaw angle and yaw rate
  double _phi, _wx; // roll angle and roll rate

  // acceleration 'states'
  double _udot, _vdot;
  double _wxdot, _wzdot;

  // vertical forces on each tire
  double _fzlf, _fzrf, _fzlr, _fzrr;
};

// sets the vertical forces based on the vehicle weight
void vehInit(VehicleState &v_state, const VehicleParam &v_params);

double driveTorque(const VehicleParam &v_params, const double throttle,
                   const double omega);

inline double brakeTorque(const VehicleParam &v_params, const double brake) {
  return v_params._maxBrakeTorque * brake;
}

// function to advance the time step of the vehicle
void vehAdv(VehicleState &v_states, const VehicleParam &v_params,
            const std::vector<double> &fx, const std::vector<double> &fy,
            const double huf, const double hur);

// setting vehicle parameters using a JSON file
void setVehParamsJSON(VehicleParam &v_params, std::string fileName);

void vehToTireTransform(TMeasyState &tirelf_st, TMeasyState &tirerf_st,
                        TMeasyState &tirelr_st, TMeasyState &tirerr_st,
                        const VehicleState &v_states,
                        const VehicleParam &v_params,
                        const std::vector<double> &controls);

void tireToVehTransform(TMeasyState &tirelf_st, TMeasyState &tirerf_st,
                        TMeasyState &tirelr_st, TMeasyState &tirerr_st,
                        const VehicleState &v_states,
                        const VehicleParam &v_params,
                        const std::vector<double> &controls);
#endif