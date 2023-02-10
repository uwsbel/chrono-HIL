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
// The TMeasy for the 8dof vehicle model
// This class includes the computation of tire dynamics for 8dof vehicle model
//
// =============================================================================

#include "rom_TMeasy.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "rom_utils.h"
#include <cmath>
#include <iostream>
#include <stdint.h>

using namespace chrono;
using namespace chrono::vehicle;

/*
Code for the TM easy tire model implemented with the 8DOF model
*/

// initialize the tire deflection based on the vehicle weight
// tempated based on which tire we get
// 0 - LF
// 1 - RF
// 2 - LR
// 3 - RR
void tireInit(TMeasyParam &t_params) {

  // calculates some critical values that are needed
  t_params._fzRdynco =
      (t_params._pn * (t_params._rdyncoP2n - 2.0 * t_params._rdyncoPn + 1.)) /
      (2. * (t_params._rdyncoP2n - t_params._rdyncoPn));

  t_params._rdyncoCrit = InterpL(t_params._fzRdynco, t_params._rdyncoPn,
                                 t_params._rdyncoP2n, t_params._pn);
}

void tmxy_combined(double &f, double &fos, double s, double df0, double sm,
                   double fm, double ss, double fs) {

  double df0loc = 0.0;
  if (sm > 0.0) {
    df0loc = std::max(2.0 * fm / sm, df0);
  }

  if (s > 0.0 && df0loc > 0.0) { // normal operating conditions
    if (s > ss) {                // full sliding
      f = fs;
      fos = f / s;
    } else {
      if (s < sm) { // adhesion
        double p = df0loc * sm / fm - 2.0;
        double sn = s / sm;
        double dn = 1.0 + (sn + p) * sn;
        f = df0loc * sm * sn / dn;
        fos = df0loc / dn;
      } else {
        double a = std::pow(fm / sm, 2.0) /
                   (df0loc * sm); // parameter from 2. deriv. of f @ s=sm
        double sstar = sm + (fm - fs) / (a * (ss - sm)); // connecting point
        if (sstar <= ss) {                               // 2 parabolas
          if (s <= sstar) {
            // 1. parabola sm < s < sstar
            f = fm - a * (s - sm) * (s - sm);
          } else {
            // 2. parabola sstar < s < ss
            double b = a * (sstar - sm) / (ss - sstar);
            f = fs + b * (ss - s) * (ss - s);
          }
        } else {
          // cubic fallback function
          double sn = (s - sm) / (ss - sm);
          f = fm - (fm - fs) * sn * sn * (3.0 - 2.0 * sn);
        }
        fos = f / s;
      }
    }
  } else {
    f = 0.0;
    fos = 0.0;
  }
}

// Advance the tire to the next time step
// update the tire forces which will be used by the vehicle
// whichTire specifies
// 0 - LF
// 1 - RF
// 2 - LR
// 3 - RR
void tireAdv(TMeasyState &t_states, const TMeasyParam &t_params,
             const VehicleState &v_states, const VehicleParam &v_params,
             const std::vector<double> &controls) {

  // get the controls and time out
  double t = controls[0];
  double delta = controls[1] * v_params._maxSteer;
  double throttle = controls[2];
  double brake = controls[3];

  // Get the whichTire based variables out of the way
  double fz = t_states._fz;   // vertical force
  double vsy = t_states._vsy; // y slip velocity
  double vsx = t_states._vsx; // x slip velocity

  // get our tire deflections so that we can get the loaded radius
  t_states._xt = fz / t_params._kt;
  t_states._rStat = t_params._r0 - t_states._xt;

  double r_eff;
  if (fz <= t_params._fzRdynco) {
    double rdynco =
        InterpL(fz, t_params._rdyncoPn, t_params._rdyncoP2n, t_params._pn);
    r_eff = rdynco * t_params._r0 + (1. - rdynco) * t_states._rStat;
  } else {
    double rdynco = t_params._rdyncoCrit;
    r_eff = rdynco * t_params._r0 + (1. - rdynco) * t_states._rStat;
  }

  // with this r_eff, we can finalize the x slip velocity
  vsx = vsx - (t_states._omega * r_eff);

  // get the transport velocity - 0.01 here is to prevent singularity
  double vta = r_eff * std::abs(t_states._omega) + 0.01;

  // evaluate the slips
  double sx = -vsx / vta;
  double alpha;
  // only front wheel steering
  alpha = std::atan2(vsy, vta) - delta;
  double sy = -std::tan(alpha);

  // limit fz
  if (fz > t_params._pnmax) {
    fz = t_params._pnmax;
  }

  // calculate all curve parameters through interpolation
  double dfx0 = InterpQ(fz, t_params._dfx0Pn, t_params._dfx0P2n, t_params._pn);
  double dfy0 = InterpQ(fz, t_params._dfy0Pn, t_params._dfy0P2n, t_params._pn);

  double fxm = InterpQ(fz, t_params._fxmPn, t_params._fxmP2n, t_params._pn);
  double fym = InterpQ(fz, t_params._fymPn, t_params._fymP2n, t_params._pn);

  double fxs = InterpQ(fz, t_params._fxsPn, t_params._fxsP2n, t_params._pn);
  double fys = InterpQ(fz, t_params._fysPn, t_params._fysP2n, t_params._pn);

  double sxm = InterpL(fz, t_params._sxmPn, t_params._sxmP2n, t_params._pn);
  double sym = InterpL(fz, t_params._symPn, t_params._symP2n, t_params._pn);

  double sxs = InterpL(fz, t_params._sxsPn, t_params._sxsP2n, t_params._pn);
  double sys = InterpL(fz, t_params._sysPn, t_params._sysP2n, t_params._pn);

  // slip normalizing factors
  double hsxn = sxm / (sxm + sym) + (fxm / dfx0) / (fxm / dfx0 + fym / dfy0);
  double hsyn = sym / (sxm + sym) + (fym / dfy0) / (fxm / dfx0 + fym / dfy0);

  // normalized slip
  double sxn = sx / hsxn;
  double syn = sy / hsyn;

  // combined slip
  double sc = std::hypot(sxn, syn);

  // cos and sine alphs
  double calpha;
  double salpha;
  if (sc > 0) {
    calpha = sxn / sc;
    salpha = syn / sc;
  } else {
    calpha = std::sqrt(2.) / 2.;
    salpha = std::sqrt(2.) / 2.;
  }

  // resultant curve parameters in both directions
  double df0 = std::hypot(dfx0 * calpha * hsxn, dfy0 * salpha * hsyn);
  double fm = std::hypot(fxm * calpha, fym * salpha);
  double sm = std::hypot(sxm * calpha / hsxn, sym * salpha / hsyn);
  double fs = std::hypot(fxs * calpha, fys * salpha);
  double ss = std::hypot(sxs * calpha / hsxn, sys * salpha / hsyn);

  // calculate force and force /slip from the curve characteritics
  double f, fos;
  tmxy_combined(f, fos, sc, df0, sm, fm, ss, fs);

  // static or "structural" force
  double Fx, Fy;
  if (sc > 0.) {
    Fx = f * sx / sc;
    Fy = f * sy / sc;
  } else {
    Fx = 0.;
    Fy = 0.;
  }

  // rolling resistance with smoothing
  double vx_min = 0;
  double vx_max = 0;

  double My = -sineStep(vta, vx_min, 0., vx_max, 1.) * t_params._rr * fz *
              t_states._rStat * sgn(t_states._omega);

  double h;
  double dOmega;

  // some normalised slip velocities
  double vtxs = vta * hsxn;
  double vtys = vta * hsyn;

  // some varables needed in the loop
  double fxdyn, fydyn;
  double fxstr, fystr;
  double v_step = v_params._step;
  double tire_step = t_params._step;
  // now we integrate to the next vehicle time step
  double tEnd = t + v_step;
  while (t < tEnd) {

    // ensure that we integrate exactly to step
    h = std::min(tire_step, tEnd - t);

    // always integrate using half implicit
    // just a placeholder to simplify the forumlae
    double dFx = -vtxs * t_params._cx / (vtxs * t_params._dx + fos);

    t_states._xedot = 1. / (1. - h * dFx) *
                      (-vtxs * t_params._cx * t_states._xe - fos * vsx) /
                      (vtxs * t_params._dx + fos);

    t_states._xe = t_states._xe + h * t_states._xedot;

    double dFy = -vtys * t_params._cy / (vtys * t_params._dy + fos);
    t_states._yedot =
        (1. / (1. - h * dFy)) *
        (-vtys * t_params._cy * t_states._ye - fos * (-sy * vta)) /
        (vtys * t_params._dy + fos);

    t_states._ye = t_states._ye + h * t_states._yedot;

    // update the force since we need to update the force to get the omegas
    // some wierd stuff happens between the dynamic and structural force
    fxdyn = t_params._dx * (-vtxs * t_params._cx * t_states._xe - fos * vsx) /
                (vtxs * t_params._dx + fos) +
            t_params._cx * t_states._xe;

    fydyn = t_params._dy *
                ((-vtys * t_params._cy * t_states._ye - fos * (-sy * vta)) /
                 (vtys * t_params._dy + fos)) +
            (t_params._cy * t_states._ye);

    fxstr = clamp(t_states._xe * t_params._cx + t_states._xedot * t_params._dx,
                  -t_params._fxmP2n, t_params._fxmP2n);
    fystr = clamp(t_states._ye * t_params._cy + t_states._yedot * t_params._dy,
                  -t_params._fymP2n, t_params._fymP2n);

    double weightx = sineStep(std::abs(vsx), 1., 1., 1.5, 0.);
    double weighty = sineStep(std::abs(-sy * vta), 1., 1., 1.5, 0.);

    // now finally get the resultant force
    t_states._fx = weightx * fxstr + (1. - weightx) * fxdyn;
    t_states._fy = weighty * fystr + (1. - weighty) * fydyn;

    // now use this force for our omegas
    dOmega = (1 / t_params._jw) *
             (driveTorque(v_params, throttle, t_states._omega) / 4. + My -
              sgn(t_states._omega) * brakeTorque(v_params, brake) -
              t_states._fx * t_states._rStat);

    // integrate omega using the latest dOmega
    t_states._omega = t_states._omega + v_params._step * dOmega;

    t += h;
  }
}

// setting Tire parameters using a JSON file
void setTireParamsJSON(TMeasyParam &t_params, std::string fileName) {

  // parse the stream into DOM tree
  rapidjson::Document d;
  vehicle::ReadFileJSON(fileName, d);

  if (d.HasParseError()) {
    std::cout << "Error with rapidjson:" << std::endl
              << d.GetParseError() << std::endl;
  }

  // pray to what ever you believe in and hope that the json file has all these
  t_params._jw = d["jw"].GetDouble();
  t_params._rr = d["rr"].GetDouble();
  t_params._r0 = d["r0"].GetDouble();
  t_params._pn = d["pn"].GetDouble();
  t_params._pnmax = d["pnmax"].GetDouble();
  t_params._cx = d["cx"].GetDouble();
  t_params._cy = d["cy"].GetDouble();
  t_params._kt = d["kt"].GetDouble();
  t_params._dx = d["dx"].GetDouble();
  t_params._dy = d["dy"].GetDouble();
  t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
  t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
  t_params._fzRdynco = d["fzRdynco"].GetDouble();
  t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble();

  t_params._dfx0Pn = d["dfx0Pn"].GetDouble();
  t_params._dfx0P2n = d["dfx0P2n"].GetDouble();
  t_params._fxmPn = d["fxmPn"].GetDouble();
  t_params._fxmP2n = d["fxmP2n"].GetDouble();
  t_params._fxsPn = d["fxsPn"].GetDouble();
  t_params._fxsP2n = d["fxsP2n"].GetDouble();
  t_params._sxmPn = d["sxmPn"].GetDouble();
  t_params._sxmP2n = d["sxmP2n"].GetDouble();
  t_params._sxsPn = d["sxsPn"].GetDouble();
  t_params._sxsP2n = d["sxsP2n"].GetDouble();

  t_params._dfy0Pn = d["dfy0Pn"].GetDouble();
  t_params._dfy0P2n = d["dfy0P2n"].GetDouble();
  t_params._fymPn = d["fymPn"].GetDouble();
  t_params._fymP2n = d["fymP2n"].GetDouble();
  t_params._fysPn = d["fysPn"].GetDouble();
  t_params._fysP2n = d["fysP2n"].GetDouble();
  t_params._symPn = d["symPn"].GetDouble();
  t_params._symP2n = d["symP2n"].GetDouble();
  t_params._sysPn = d["sysPn"].GetDouble();
  t_params._sysP2n = d["sysP2n"].GetDouble();

  t_params._step = d["step"].GetDouble();
}
