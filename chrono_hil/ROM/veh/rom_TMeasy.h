#ifndef TMEASY_H
#define TMEASY_H

#include "rom_Eightdof.h"
#include <stdint.h>

/*
Headerfor the TM easy tire model implemented with the 8DOF model
*/

// TMeasy parameter structure
struct TMeasyParam {

  // constructor that takes default values of HMMWV
  TMeasyParam()
      : _jw(6.69), _rr(0.015), _mu(0.8), _r0(0.4699), _pn(8562.8266),
        _pnmax(29969.893), _cx(185004.42), _cy(164448.37), _kt(411121.0),
        _dx(3700.), _dy(3488.), _rdyncoPn(0.375), _rdyncoP2n(0.75),
        _fzRdynco(0), _rdyncoCrit(0), _dfx0Pn(151447.29), _dfx0P2n(236412.79),
        _fxmPn(7575.3614), _fxmP2n(12808.276), _fxsPn(4657.9208),
        _fxsP2n(8625.3352), _sxmPn(0.12), _sxmP2n(0.15), _sxsPn(0.9),
        _sxsP2n(0.95), _dfy0Pn(50931.693), _dfy0P2n(94293.847),
        _fymPn(6615.0404), _fymP2n(12509.947), _fysPn(6091.5092),
        _fysP2n(11443.875), _symPn(0.38786), _symP2n(0.38786), _sysPn(0.82534),
        _sysP2n(0.91309), _step(1e-2) {}

  // constructor that takes given values - ugly looking code - can this be
  // beutified?
  TMeasyParam(double jw, double rr, double mu, double r0, double pn,
              double pnmax, double cx, double cy, double dx, double dy,
              double kt, double rdyncoPn, double rdyncoP2n, double fzRdynco,
              double rdyncoCrit, double dfx0Pn, double dfx0P2n, double fxmPn,
              double fxmP2n, double fxsPn, double fxsP2n, double sxmPn,
              double sxmP2n, double sxsPn, double sxsP2n, double dfy0Pn,
              double dfy0P2n, double fymPn, double fymP2n, double fysPn,
              double fysP2n, double symPn, double symP2n, double sysPn,
              double sysP2n, double step)
      : _jw(jw), _rr(rr), _mu(mu), _r0(r0), _pn(pn), _pnmax(pnmax), _cx(cx),
        _cy(cy), _kt(kt), _dx(dx), _dy(dy), _rdyncoPn(rdyncoPn),
        _rdyncoP2n(rdyncoP2n), _fzRdynco(fzRdynco), _rdyncoCrit(rdyncoCrit),
        _dfx0Pn(dfx0Pn), _dfx0P2n(dfx0P2n), _fxmPn(fxmPn), _fxmP2n(fxmP2n),
        _fxsPn(fxsPn), _fxsP2n(fxsP2n), _sxmPn(sxmPn), _sxmP2n(sxmP2n),
        _sxsPn(sxsPn), _sxsP2n(sxsP2n), _dfy0Pn(dfy0Pn), _dfy0P2n(dfy0P2n),
        _fymPn(fymPn), _fymP2n(fymP2n), _fysPn(fysPn), _fysP2n(fysP2n),
        _symPn(symPn), _symP2n(symP2n), _sysPn(sysPn), _sysP2n(sysP2n),
        _step(step) {}

  // basic tire parameters
  double _jw; // wheel inertia
  double _rr; // rolling resistance of tire
  double _mu; // friction constant
  double _r0; // unloaded tire radius

  // TM easy specific tire params
  double _pn, _pnmax;   // nominal and max vertical force
  double _cx, _cy, _kt; // longitudinal, lateral and vertical stiffness
  double _dx,
      _dy; // longitudinal and lateral damping coeffs. No vertical damping

  // TM easy force characteristic params
  // 2 values - one at nominal load and one at max load

  // dynamic radius weighting coefficient and a critical value for the vertical
  // force
  double _rdyncoPn, _rdyncoP2n, _fzRdynco, _rdyncoCrit;

  // Longitudinal
  double _dfx0Pn, _dfx0P2n; // intial longitudinal slopes dFx/dsx [N]
  double _fxmPn, _fxmP2n;   // maximum longituidnal force [N]
  double _fxsPn, _fxsP2n;   // Longitudinal load at sliding [N]
  double _sxmPn, _sxmP2n;   // slip sx at maximum longitudinal load Fx
  double _sxsPn, _sxsP2n;   // slip sx where sliding begins

  // Lateral
  double _dfy0Pn, _dfy0P2n; // intial lateral slopes dFx/dsx [N]
  double _fymPn, _fymP2n;   // maximum lateral force [N]
  double _fysPn, _fysP2n;   // Lateral load at sliding [N]
  double _symPn, _symP2n;   // slip sx at maximum lateral load Fx
  double _sysPn, _sysP2n;   // slip sx where sliding begins

  double _step; // integration time step
};

// Tm easy state structure - actual states + things that we need to keep track
// of
struct TMeasyState {
  // default contructor to 0's
  TMeasyState()
      : _xe(0.), _ye(0.), _xedot(0.), _yedot(0.), _omega(0.), _xt(0.),
        _rStat(0.), _fx(0.), _fy(0.), _fz(0.), _vsx(0.), _vsy(0.) {}

  // special constructor in case we want to start the simualtion at
  // some other time step
  TMeasyState(double xe, double ye, double xedot, double yedot, double omega,
              double xt, double rStat, double fx, double fy, double fz,
              double vsx, double vsy)
      : _xe(xe), _ye(ye), _xedot(xedot), _yedot(yedot), _omega(omega), _xt(xt),
        _rStat(rStat), _fx(fx), _fy(fy), _fz(fz), _vsx(vsx), _vsy(vsy) {}

  // the actual state that are intgrated
  double _xe, _ye;       // long and lat tire deflection
  double _xedot, _yedot; // long and lat tire deflection velocity
  double _omega;         // angular velocity of wheel

  // other "states" that we need to keep track of
  double _xt;           // vertical tire compression
  double _rStat;        // loaded tire radius
  double _fx, _fy, _fz; // long, lateral and vertical force in tire frame

  // velocities in tire frame
  double _vsx, _vsy;
};

// sets the vertical tire deflection based on the vehicle weight
// template based on which tire
void tireInit(TMeasyParam &t_params);

// function to calculate the force from the force charactristics
// used by tireSync
void tmxy_combined(double &f, double &fos, double s, double df0, double sm,
                   double fm, double ss, double fs);

// Advances the time step
// updates the tire forces which is then used by the vehicle model
void tireAdv(TMeasyState &t_states, const TMeasyParam &t_params,
             const VehicleState &v_states, const VehicleParam &v_params,
             const std::vector<double> &controls);

// setting tire parameters using a JSON file
void setTireParamsJSON(TMeasyParam &t_params, std::string fileName);

#endif