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
// Author: Jason Zhou
// =============================================================================

#include <chrono>
#include <iostream>
#include <stdint.h>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_hil/driver/ChSDLInterface.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include <irrlicht.h>

#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_hil/ROM/driver/ChROM_IDMFollower.h"
#include "chrono_hil/ROM/driver/ChROM_PathFollowerDriver.h"
#include "chrono_hil/ROM/veh/Ch_8DOF_vehicle.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono/core/ChBezierCurve.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

#include "chrono_hil/timer/ChRealtimeCumulative.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;
using namespace chrono::hil;

// Simulation step sizes
double step_size = 5e-4;
double tire_step_size = 1e-3;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50; // FPS = 50

ChVector<> initLoc(0, 0, 1.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

int main(int argc, char *argv[]) {
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));
  // ========== Chrono::Vehicle HMMWV vehicle ===============
  // Create the HMMWV vehicle, set parameters, and initialize
  HMMWV_Full my_hmmwv;
  my_hmmwv.SetContactMethod(ChContactMethod::SMC);
  my_hmmwv.SetChassisCollisionType(CollisionType::NONE);
  my_hmmwv.SetChassisFixed(false);
  my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
  my_hmmwv.SetDriveType(DrivelineTypeWV::AWD);
  my_hmmwv.UseTierodBodies(true);
  my_hmmwv.SetSteeringType(SteeringTypeWV::PITMAN_ARM);
  my_hmmwv.SetBrakeType(BrakeType::SHAFTS);
  my_hmmwv.SetTireType(TireModelType::TMEASY);
  my_hmmwv.SetTireStepSize(tire_step_size);
  my_hmmwv.Initialize();

  my_hmmwv.SetChassisVisualizationType(VisualizationType::MESH);
  my_hmmwv.SetSuspensionVisualizationType(VisualizationType::MESH);
  my_hmmwv.SetSteeringVisualizationType(VisualizationType::MESH);
  my_hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
  my_hmmwv.SetTireVisualizationType(VisualizationType::MESH);

  // ROM HMMWV model
  std::string hmmwv_rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";
  std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
      chrono_types::make_shared<Ch_8DOF_vehicle>(hmmwv_rom_json, 0.45);
  rom_veh->SetInitPos(initLoc + ChVector<>(0.0, 4.0, 0.45));
  rom_veh->SetInitRot(0.0);
  rom_veh->Initialize(my_hmmwv.GetSystem());

  // Initialize terrain
  RigidTerrain terrain(my_hmmwv.GetSystem());

  double terrainLength = 200.0; // size in X direction
  double terrainWidth = 200.0;  // size in Y direction

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
  std::shared_ptr<RigidTerrain::Patch> patch;
  patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
  patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 200,
                    200);
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
  terrain.Initialize();

  // Create a body that camera attaches to
  auto attached_body = std::make_shared<ChBody>();
  my_hmmwv.GetSystem()->AddBody(attached_body);
  attached_body->SetPos(ChVector<>(0.0, 0.0, 0.0));
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // Create camera
  // Create the camera sensor
  auto manager =
      chrono_types::make_shared<ChSensorManager>(my_hmmwv.GetSystem());
  float intensity = 1.2;
  manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  auto cam = chrono_types::make_shared<ChCameraSensor>(
      attached_body, // body camera is attached to
      35,            // update rate in Hz
      chrono::ChFrame<double>(
          ChVector<>(15.0, -25.0, 10.0),
          Q_from_Euler123(ChVector<>(0.0, C_PI / 6, C_PI / 2))), // offset pose
      1280,                                                      // image width
      720,                                                       // image
      1.608f, 1); // fov, lag, exposure cam->SetName("Camera Sensor");

  cam->PushFilter(
      chrono_types::make_shared<ChFilterVisualize>(1280, 720, "test", false));
  // Provide the host access to the RGBA8 buffer
  // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(cam);

  manager->Update();

  // Set the time response for steering and throttle keyboard inputs.
  double steering_time = 1.0; // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0; // time to go from 0 to +1
  double braking_time = 0.3;  // time to go from 0 to +1

  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(render_step_size / step_size);

  // Initialize simulation frame counters
  int step_number = 0;
  int render_frame = 0;
  double time = 0.0;

  ChRealtimeCumulative realtime_timer;

  while (time < t_end) {

    if (step_number == 0) {
      realtime_timer.Reset();
    }

    time = my_hmmwv.GetSystem()->GetChTime();

    // End simulation
    if (time >= t_end)
      break;

    // Driver inputs
    DriverInputs driver_inputs;

    // Time-based drive inputs
    if (time < 3.0f) {
      driver_inputs.m_throttle = 0.0;
      driver_inputs.m_braking = 0.0;
      driver_inputs.m_steering = 0.0;
    } else if (time >= 3.0f && time < 8.0f) {
      driver_inputs.m_throttle = 0.5;
      driver_inputs.m_braking = 0.0;
      driver_inputs.m_steering = 0.0;
    } else if (time >= 8.0f && time < 12.0f) {
      driver_inputs.m_throttle = 0.0;
      driver_inputs.m_braking = 0.4;
      driver_inputs.m_steering = 0.0;
    } else {
      driver_inputs.m_throttle = 0.0;
      driver_inputs.m_braking = 0.0;
      driver_inputs.m_steering = 0.0;
    }

    // Update modules (process inputs from other modules)
    terrain.Synchronize(time);
    my_hmmwv.Synchronize(time, driver_inputs, terrain);
    // Advance simulation for one timestep for all modules
    terrain.Advance(step_size);
    my_hmmwv.Advance(step_size);
    rom_veh->Advance(time, driver_inputs);

    manager->Update();

    // Increment frame number
    step_number++;

    realtime_timer.Spin(time);
  }
}