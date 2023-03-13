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

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/network/ChBoostInStreamer.h"
#include "chrono_hil/network/ChBoostOutStreamer.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;
using namespace chrono::geometry;
using namespace chrono::sensor;
using namespace chrono::hil;

const double RADS_2_RPM = 30 / CH_C_PI;
const double MS_2_MPH = 2.2369;

#define PORT_IN 1209
#define PORT_OUT 1204
#define IP_OUT "128.104.190.70"
bool render = true;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(-91.788, 98.647, 0.4);
ChQuaternion<> initRot(1, 0, 0, 0);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-5;

// Simulation end time
double t_end = 1000;

// =============================================================================

int main(int argc, char *argv[]) {
  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: "
           << CHRONO_VERSION << "\n\n";

  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  std::string vehicle_filename =
      vehicle::GetDataFile("audi/json/audi_Vehicle.json");
  std::string powertrain_filename =
      vehicle::GetDataFile("audi/json/audi_SimpleMapPowertrain.json");
  std::string tire_filename =
      vehicle::GetDataFile("audi/json/audi_Pac02Tire.json");

  // --------------
  // Create systems
  // --------------

  // Create the Sedan vehicle, set parameters, and initialize
  WheeledVehicle my_vehicle(vehicle_filename, ChContactMethod::SMC);
  auto ego_chassis = my_vehicle.GetChassis();
  my_vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
  my_vehicle.GetChassis()->SetFixed(false);
  auto powertrain = ReadPowertrainJSON(powertrain_filename);
  my_vehicle.InitializePowertrain(powertrain);
  my_vehicle.SetChassisVisualizationType(VisualizationType::MESH);
  my_vehicle.SetSuspensionVisualizationType(VisualizationType::MESH);
  my_vehicle.SetSteeringVisualizationType(VisualizationType::MESH);
  my_vehicle.SetWheelVisualizationType(VisualizationType::MESH);

  // Create and initialize the tires
  for (auto &axle : my_vehicle.GetAxles()) {
    for (auto &wheel : axle->GetWheels()) {
      auto tire = ReadTireJSON(tire_filename);
      tire->SetStepsize(tire_step_size);
      my_vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
    }
  }

  auto attached_body = std::make_shared<ChBody>();
  my_vehicle.GetSystem()->AddBody(attached_body);
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // Create the terrain
  RigidTerrain terrain(my_vehicle.GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;

  patch = terrain.AddPatch(patch_mat, CSYSNORM,
                           std::string(STRINGIFY(HIL_DATA_DIR)) +
                               "/Environments/nads/newnads/terrain.obj");

  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain.Initialize();

  // add vis mesh
  auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  terrain_mesh->LoadWavefrontMesh(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                      "/Environments/nads/newnads/terrain.obj",
                                  true, true);
  terrain_mesh->Transform(ChVector<>(0, 0, 0),
                          ChMatrix33<>(1)); // scale to a different size
  auto terrain_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  terrain_shape->SetMesh(terrain_mesh);
  terrain_shape->SetName("terrain");
  terrain_shape->SetMutable(false);

  auto terrain_body = chrono_types::make_shared<ChBody>();
  terrain_body->SetPos({0, 0, -.01});
  // terrain_body->SetRot(Q_from_AngX(CH_C_PI_2));
  terrain_body->AddVisualShape(terrain_shape);
  terrain_body->SetBodyFixed(true);
  terrain_body->SetCollide(false);
  my_vehicle.GetSystem()->Add(terrain_body);

  // ------------------------
  // Create the driver system
  // ------------------------

  ChBoostInStreamer in_streamer(PORT_IN, 4);
  std::vector<float> recv_data;

  // ---------------
  // Simulation loop
  // ---------------
  std::cout << "\nVehicle mass: " << my_vehicle.GetMass() << std::endl;

  // Initialize simulation frame counters
  int step_number = 0;

  // Create the camera sensor
  auto manager =
      chrono_types::make_shared<ChSensorManager>(my_vehicle.GetSystem());
  if (render) {
    float intensity = 1.2;
    manager->scene->AddPointLight({0, 0, 1e8}, {1.0, 1.0, 1.0}, 1e12);
    manager->scene->SetAmbientLight({.2, .2, .2});
    manager->scene->SetSceneEpsilon(1e-3);
    manager->scene->EnableDynamicOrigin(true);
    manager->scene->SetOriginOffsetThreshold(500.f);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        attached_body, // body camera is attached to
        30,            // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(-12.0, 0.0, 2.0),
            Q_from_Euler123(ChVector<>(0.0, 0.11, 0.0))), // offset pose
        5760,                                             // image width
        1080,                                             // image height
        1.408f,
        2); // fov, lag, exposure
    cam->SetName("Camera Sensor");

    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
        1280, 720, "hwwmv", false));
    // Provide the host access to the RGBA8 buffer
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);
  }

  my_vehicle.EnableRealtime(false);

  ChRealtimeCumulative realtime_timer;
  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double last_time = 0;

  // create boost data streaming interface
  ChBoostOutStreamer boost_streamer(IP_OUT, PORT_OUT);

  // simulation loop
  while (true) {
    double time = my_vehicle.GetSystem()->GetChTime();

    ChVector<> pos = my_vehicle.GetChassis()->GetPos();
    ChQuaternion<> rot = my_vehicle.GetChassis()->GetRot();

    auto euler_rot = Q_to_Euler123(rot);
    euler_rot.x() = 0.0;
    euler_rot.y() = 0.0;
    auto y_0_rot = Q_from_Euler123(euler_rot);

    attached_body->SetPos(pos);
    attached_body->SetRot(y_0_rot);

    if (render) {
      manager->Update();
    }

    // End simulation
    if (time >= t_end)
      break;

    // Get driver inputs
    DriverInputs driver_inputs;

    in_streamer.Synchronize();
    recv_data = in_streamer.GetRecvData();
    driver_inputs.m_throttle = recv_data[0];
    driver_inputs.m_steering = recv_data[1];
    driver_inputs.m_braking = recv_data[2];

    float gear = recv_data[3];
    std::cout << gear << std::endl;
    if (gear == 0.0) {
      my_vehicle.GetPowertrain()->SetDriveMode(
          ChPowertrain::DriveMode::NEUTRAL);
      driver_inputs.m_braking = 1.0;
    } else if (gear == 1.0) {
      my_vehicle.GetPowertrain()->SetDriveMode(
          ChPowertrain::DriveMode::FORWARD);
    } else if (gear == 2.0) {
      my_vehicle.GetPowertrain()->SetDriveMode(
          ChPowertrain::DriveMode::REVERSE);
    } else if (gear == 3.0) {
      my_vehicle.GetPowertrain()->SetDriveMode(
          ChPowertrain::DriveMode::NEUTRAL);
    }

    // =======================
    // data stream out section
    // =======================
    boost_streamer.AddData((float)time); // 0 - time
    boost_streamer.AddData(pos.x());     // 1 - x position
    boost_streamer.AddData(pos.y());     // 2 - y position
    boost_streamer.AddData(pos.z());     // 3 - z position
    auto eu_rot = Q_to_Euler123(rot);
    boost_streamer.AddData(eu_rot.x()); // 4 - x rotation
    boost_streamer.AddData(eu_rot.y()); // 5 - y rotation
    boost_streamer.AddData(eu_rot.z()); // 6 - z rotation
    auto vel =
        my_vehicle.GetChassis()->GetBody()->GetFrame_REF_to_abs().GetPos_dt();
    boost_streamer.AddData(vel.x()); // 7 - x velocity
    boost_streamer.AddData(vel.y()); // 8 - y velocity
    boost_streamer.AddData(vel.z()); // 9 - z velocity
    boost_streamer.AddData(
        (float)(my_vehicle.GetSpeed() * MS_2_MPH)); // 10 - speed (m/s)

    auto acc =
        my_vehicle.GetChassis()->GetBody()->GetFrame_REF_to_abs().GetPos_dtdt();
    boost_streamer.AddData(acc.x()); // 11 - x acceleration
    boost_streamer.AddData(acc.y()); // 12 - y acceleration
    boost_streamer.AddData(acc.z()); // 13 - z acceleration

    auto ang_vel_q =
        my_vehicle.GetChassis()->GetBody()->GetFrame_REF_to_abs().GetRot_dt();
    auto ang_vel = Q_to_Euler123(ang_vel_q);
    boost_streamer.AddData(ang_vel.x()); // 14 - x ang vel of chassis
    boost_streamer.AddData(ang_vel.y()); // 15 - y ang vel of chassis
    boost_streamer.AddData(ang_vel.z()); // 16 - z ang vel of chassis

    boost_streamer.AddData(
        my_vehicle.GetPowertrain()
            ->GetCurrentTransmissionGear()); // 17 - current gear
    boost_streamer.AddData(my_vehicle.GetPowertrain()->GetMotorSpeed() *
                           RADS_2_RPM); // 18 - current RPM
    boost_streamer.Synchronize();
    // =======================
    // end data stream out section
    // =======================

    // Update modules (process inputs from other modules)
    terrain.Synchronize(time);
    my_vehicle.Synchronize(time, driver_inputs, terrain);

    // Advance simulation for one timestep for all modules
    terrain.Advance(step_size);
    my_vehicle.Advance(step_size);

    // Increment frame number
    step_number++;

    if (step_number == 0) {
      realtime_timer.Reset();
    }

    if (step_number % 10 == 0) {
      realtime_timer.Spin(time);
    }

    if (step_number % 500 == 0) {
      std::chrono::high_resolution_clock::time_point end =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> wall_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                    start);

      std::cout << (wall_time.count()) / (time - last_time) << "\n";
      last_time = time;
      start = std::chrono::high_resolution_clock::now();
    }
  }

  return 0;
}
