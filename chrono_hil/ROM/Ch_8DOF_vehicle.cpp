#include "Ch_8DOF_vehicle.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;

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

  chassis_body->SetPos(this->GetPos());

  chassis_body->SetRot(this->GetRot());

  ChFrame<> chassis_body_fr = ChFrame<>(this->GetPos(), this->GetRot());

  ChFrame<> X_LF =
      chassis_body_fr * ChFrame<>(wheels_offset_pos[0], wheels_offset_rot[0]);
  ChFrame<> X_RF =
      chassis_body_fr * ChFrame<>(wheels_offset_pos[1], wheels_offset_rot[1]);
  ChFrame<> X_LR =
      chassis_body_fr * ChFrame<>(wheels_offset_pos[2], wheels_offset_rot[2]);
  ChFrame<> X_RR =
      chassis_body_fr * ChFrame<>(wheels_offset_pos[3], wheels_offset_rot[3]);

  // front tire - 1 - vehicle rotation
  ChQuaternion<> f_steer_rot = chassis_body_fr.GetRot();

  // front tire - 2 - steer rotation
  ChQuaternion<> temp = ChQuaternion<>(1, 0, 0, 0);
  temp.Q_from_AngZ(inputs.m_steering * veh1_param._maxSteer);
  f_steer_rot = f_steer_rot * temp;

  // front tire - 3 - take into tire rotation
  temp = ChQuaternion<>(1, 0, 0, 0);
  temp.Q_from_AngY(prev_tire_rotation +
                   veh1_param._step * (tirelf_st._omega / 180.f * C_PI));
  prev_tire_rotation =
      prev_tire_rotation + veh1_param._step * (tirelf_st._omega / 180.f * C_PI);
  if (prev_tire_rotation > C_2PI) {
    prev_tire_rotation = 0.f;
  }
  f_steer_rot = f_steer_rot * temp;

  // rear tire - 1 - vehicle rotation
  ChQuaternion<> r_steer_rot = chassis_body_fr.GetRot();
  temp = ChQuaternion<>(1, 0, 0, 0);
  temp.Q_from_AngY(prev_tire_rotation +
                   veh1_param._step * (tirelf_st._omega / 180.f * C_PI));
  r_steer_rot = r_steer_rot * temp;

  prev_tire_rotation =
      prev_tire_rotation + veh1_param._step * (tirelf_st._omega / 180.f * C_PI);
  if (prev_tire_rotation > C_2PI) {
    prev_tire_rotation = 0.f;
  }

  for (int i = 0; i < 4; i++) {
    if (i == 0) {
      wheels_body[i]->SetPos(X_LF.GetPos());
      wheels_body[i]->SetRot(f_steer_rot);
    }

    if (i == 1) {
      wheels_body[i]->SetPos(X_RF.GetPos());
      wheels_body[i]->SetRot(f_steer_rot);
    }
    if (i == 2) {
      wheels_body[i]->SetPos(X_LR.GetPos());
      wheels_body[i]->SetRot(r_steer_rot);
    }
    if (i == 3) {
      wheels_body[i]->SetPos(X_RR.GetPos());
      wheels_body[i]->SetRot(r_steer_rot);
    }
  }
}

void Ch_8DOF_vehicle::InitializeVisualization(std::string chassis_obj_path,
                                              std::string wheel_obj_path,
                                              ChSystem *sys) {
  auto chassis_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  chassis_mmesh->LoadWavefrontMesh(chassis_obj_path, false, true);
  chassis_mmesh->RepairDuplicateVertexes(1e-9);

  auto chassis_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  chassis_trimesh_shape->SetMesh(chassis_mmesh);
  chassis_trimesh_shape->SetMutable(false);

  chassis_body = chrono_types::make_shared<ChBodyAuxRef>();

  chassis_body->SetCollide(false);

  chassis_body->SetBodyFixed(true);
  chassis_body->AddVisualShape(chassis_trimesh_shape);

  sys->AddBody(chassis_body);

  wheels_offset_pos[0] = ChVector<>(1.6, 1.0, 0.0);   // LF
  wheels_offset_pos[1] = ChVector<>(1.6, -1.0, 0.0);  // RF
  wheels_offset_pos[2] = ChVector<>(-1.8, 1.0, 0.0);  // LR
  wheels_offset_pos[3] = ChVector<>(-1.8, -1.0, 0.0); // RR

  // Express relative frame in global
  ChFrame<> X_LF = chassis_body->GetFrame_REF_to_abs() *
                   ChFrame<>(wheels_offset_pos[0], wheels_offset_rot[0]);
  ChFrame<> X_RF = chassis_body->GetFrame_REF_to_abs() *
                   ChFrame<>(wheels_offset_pos[1], wheels_offset_rot[1]);
  ChFrame<> X_LR = chassis_body->GetFrame_REF_to_abs() *
                   ChFrame<>(wheels_offset_pos[2], wheels_offset_rot[2]);
  ChFrame<> X_RR = chassis_body->GetFrame_REF_to_abs() *
                   ChFrame<>(wheels_offset_pos[3], wheels_offset_rot[3]);

  for (int i = 0; i < 4; i++) {
    auto wheel_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    wheel_mmesh->LoadWavefrontMesh(wheel_obj_path, false, true);
    wheel_mmesh->RepairDuplicateVertexes(1e-9);

    auto wheel_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    wheel_trimesh_shape->SetMesh(wheel_mmesh);
    wheel_trimesh_shape->SetMutable(false);

    wheels_body[i] = chrono_types::make_shared<ChBodyAuxRef>();
    wheels_body[i]->SetCollide(false);

    wheels_body[i]->SetBodyFixed(true);
    wheels_body[i]->AddVisualShape(wheel_trimesh_shape);

    if (i == 0) {
      wheels_body[i]->SetPos(X_LF.GetPos());
      wheels_body[i]->SetRot(X_LF.GetRot());
    }

    if (i == 1) {
      wheels_body[i]->SetPos(X_RF.GetPos());
      wheels_body[i]->SetRot(X_RF.GetRot());
    }
    if (i == 2) {
      wheels_body[i]->SetPos(X_LR.GetPos());
      wheels_body[i]->SetRot(X_LR.GetRot());
    }
    if (i == 3) {
      wheels_body[i]->SetPos(X_RR.GetPos());
      wheels_body[i]->SetRot(X_RR.GetRot());
    }

    sys->AddBody(wheels_body[i]);
  }
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

std::shared_ptr<ChBodyAuxRef> Ch_8DOF_vehicle::GetChassisBody() {
  return chassis_body;
}
