#include "Ch_8DOF_vehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

Ch_8DOF_vehicle::Ch_8DOF_vehicle(std::string vehicle_json,
                                 std::string tire_json, float z_plane) {

  rom_z_plane = z_plane;

  // Set vehicle parameters from JSON file
  setVehParamsJSON(veh1_param, vehicle_json);
  vehInit(veh1_st, veh1_param);

  // set the tire parameters from a JSON file
  setTireParamsJSON(tire_param, tire_json);

  // now we initialize each of our parameters
  tireInit(tire_param);
}

void Ch_8DOF_vehicle::Advance(float time, DriverInputs inputs) {
  std::vector<double> controls(4, 0);

  controls[0] = time;
  controls[1] = inputs.m_steering;
  controls[2] = inputs.m_throttle;
  controls[3] = inputs.m_braking;

  // transform velocities and other needed quantities from
  // vehicle frame to tire frame
  vehToTireTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh1_st,
                     veh1_param, controls);

  // advance our 4 tires
  tireAdv(tirelf_st, tire_param, veh1_st, veh1_param, controls);
  tireAdv(tirerf_st, tire_param, veh1_st, veh1_param, controls);

  // modify controls for our rear tires as they dont take steering
  std::vector<double> mod_controls = {controls[0], 0, controls[2], controls[3]};
  tireAdv(tirelr_st, tire_param, veh1_st, veh1_param, mod_controls);
  tireAdv(tirerr_st, tire_param, veh1_st, veh1_param, mod_controls);

  // transform tire forces to vehicle frame
  tireToVehTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh1_st,
                     veh1_param, controls);

  // copy the useful stuff that needs to be passed onto the vehicle
  std::vector<double> fx = {tirelf_st._fx, tirerf_st._fx, tirelr_st._fx,
                            tirerr_st._fx};
  std::vector<double> fy = {tirelf_st._fy, tirerf_st._fy, tirelr_st._fy,
                            tirerr_st._fy};
  double huf = tirelf_st._rStat;
  double hur = tirerr_st._rStat;

  vehAdv(veh1_st, veh1_param, fx, fy, huf, hur);
}

ChVector<> Ch_8DOF_vehicle::GetPos() {
  return ChVector<>(veh1_st._x, veh1_st._y, rom_z_plane);
}

ChQuaternion<> Ch_8DOF_vehicle::GetRot() {
  ChQuaternion ret_rot = ChQuaternion<>(1, 0, 0, 0);
  ret_rot.Q_from_Euler123(ChVector<>(veh1_st._phi, 0, veh1_st._psi));
  return ret_rot;
}

float Ch_8DOF_vehicle::GetStepSize() { return veh1_param._step; }
