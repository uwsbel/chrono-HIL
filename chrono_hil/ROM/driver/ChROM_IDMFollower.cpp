#include "ChROM_IDMFollower.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
namespace chrono {
namespace hil {

void ChROM_IDMFollower::Synchronize(double time, double step,
                                    double lead_distance, double lead_speed) {
  // In this portion we adjust the target speed according to custom piece-wise
  // sinusoidal defined in behavior_data. We use the driver model explained
  // here, using a desired speed instead:
  // https://traffic-simulation.de/info/info_IDM.html the parameters are: start
  // [miles], end [miles], v0 desired v [m/s], T desired time headway [s],
  // desired space headway [m], a: accel reate a [m/s^2], b: comfort decel
  // [m/s^2], delta: accel exponent
  dist += (m_rom->GetPos() - previousPos).Length();
  previousPos = m_rom->GetPos();

  double s = lead_distance - m_params[6];
  double v = (m_rom->GetVel()).Length();
  double delta_v = v - lead_speed;

  double s_star =
      m_params[2] +
      ChMax(0.0, v * m_params[1] +
                     (v * delta_v) / (2 * sqrt(m_params[3] * m_params[4])));
  double dv_dt = m_params[3] *
                 (1 - pow(v / m_params[0], m_params[5]) - pow(s_star / s, 2));

  // integrate intended acceleration into theoretical soeed
  thero_speed = thero_speed + dv_dt * step;
  double v_ms = ChMax(0.0, thero_speed);

  // to avoid large negative value during self drive
  if (thero_speed < 0) {
    thero_speed = 0;
  }

  m_path_follower->SetCruiseSpeed(v_ms);

  m_path_follower->Advance(step);
}

} // end namespace hil
} // end namespace chrono