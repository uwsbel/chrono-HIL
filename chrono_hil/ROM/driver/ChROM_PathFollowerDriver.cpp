#include "ChROM_PathFollowerDriver.h"
ChROM_PathFollowerDriver::ChROM_PathFollowerDriver(
    std::shared_ptr<Ch_8DOF_vehicle> rom, std::shared_ptr<ChBezierCurve> curve,
    float target_speed, double look_ahead_dist, float PID_st_kp,
    float PID_st_ki, float PID_st_kd, float PID_sp_kp, float PID_sp_ki,
    float PID_sp_kd) {
  m_dist = look_ahead_dist;
  m_target_speed = target_speed;

  // steering PID controller parameter
  m_st_kp = PID_st_kp;
  m_st_ki = PID_st_ki;
  m_st_kd = PID_st_kd;

  // speed PID controller parameter
  m_sp_kp = PID_sp_kp;
  m_sp_ki = PID_sp_ki;
  m_st_kd = PID_sp_kd;

  m_curve = curve;
  m_rom = rom;

  m_tracker = chrono_types::make_shared<ChBezierCurveTracker>(m_curve);
}

DriverInputs ChROM_PathFollowerDriver::GetDriverInput() { return m_inputs; }

void ChROM_PathFollowerDriver::Advance(float time_step) {
  ChVector<> cur_pos = m_rom->GetPos(); // current vehicle position
  ChVector<> cur_vel = m_rom->GetVel(); // current vehicle velocity

  // control steering of the vehicle
  ChVector<> sentinel =
      m_rom->GetChassisBody()
          ->GetFrame_REF_to_abs()
          .TransformPointLocalToParent(m_dist * ChWorldFrame::Forward());

  ChVector<> target;

  m_tracker->calcClosestPoint(sentinel, target);

  ChVector<> sentinel_vec = sentinel - cur_pos;
  ChWorldFrame::Project(sentinel_vec);
  ChVector<> target_vec = target - cur_pos;
  ChWorldFrame::Project(target_vec);

  ChVector<> err_vec = target - sentinel;

  float temp = Vdot(Vcross(sentinel_vec, target_vec), ChWorldFrame::Vertical());

  double st_err = ChSignum(temp) * err_vec.Length();

  m_st_err_d = (st_err - m_st_err) / time_step;
  m_st_err_i += (st_err + m_st_err) * time_step / 2;
  m_st_err = st_err;

  m_inputs.m_steering =
      m_st_kp * m_st_err + m_st_ki * m_st_err_i + m_st_kd * m_st_err_d;

  ChClampValue(m_inputs.m_steering, -1.0, 1.0);

  // control speed of the vehicle
  float cur_speed = cur_vel.Length();

  double sp_err = m_target_speed - cur_speed;

  m_sp_err_d = (sp_err - m_sp_err) / time_step;

  m_sp_err_i += (sp_err + m_sp_err) * time_step / 2;

  m_sp_err = sp_err; // cache new speed error

  double sp_res =
      m_sp_kp * m_sp_err + m_sp_ki * m_sp_err_i + m_sp_kd * m_sp_err_d;

  if (sp_res < 0) {
    double temp = abs(sp_res);
    ChClampValue(temp, 0.0, 1.0);
    m_inputs.m_braking = temp;
    m_inputs.m_throttle = 0.0;
  } else {
    double temp = abs(sp_res);
    ChClampValue(temp, 0.0, 1.0);
    m_inputs.m_braking = 0.0;
    m_inputs.m_throttle = temp;
  }
}

void ChROM_PathFollowerDriver::SetCruiseSpeed(float target_speed) {
  m_target_speed = target_speed;
}