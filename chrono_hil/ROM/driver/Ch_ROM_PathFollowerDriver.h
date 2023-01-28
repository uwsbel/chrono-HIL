
#ifndef CH_ROM_PFDRIVER_H
#define CH_ROM_PFDRIVER_H

#include "../../ChApiHil.h"
#include "../veh/Ch_8DOF_vehicle.h"
#include "chrono/core/ChBezierCurve.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChWorldFrame.h"

#include <string>

using namespace chrono;
using namespace chrono::vehicle;

class Ch_ROM_PathFollowerDriver {

public:
  Ch_ROM_PathFollowerDriver(std::shared_ptr<Ch_8DOF_vehicle> m_rom,
                            std::shared_ptr<ChBezierCurve> curve,
                            ChVector<> vertical_up, float target_speed,
                            double look_ahead_dist, float PID_st_kp,
                            float PID_st_ki, float PID_st_kd, float PID_sp_kp,
                            float PID_sp_ki, float PID_sp_kd);

  void Advance(float step);
  DriverInputs GetDriverInput();

private:
  std::shared_ptr<ChBezierCurve> m_curve;
  std::shared_ptr<ChBezierCurveTracker> m_tracker;
  std::shared_ptr<Ch_8DOF_vehicle> m_rom;

  // target spped
  float m_target_speed;

  // steering PID control
  float m_st_kp;
  float m_st_ki;
  float m_st_kd;

  // speed PID control
  float m_sp_kp;
  float m_sp_ki;
  float m_sp_kd;

  // steering errs
  float m_st_err;   // cached error
  float m_st_err_d; // d_err
  float m_st_err_i; // i_err

  // speeed errs
  float m_sp_err;   // cached error
  float m_sp_err_d; // d_err
  float m_sp_err_i; // i_err

  // look ahead distance
  double m_dist;
  ChVector<> m_up_vec;

  DriverInputs m_inputs;
};

#endif
