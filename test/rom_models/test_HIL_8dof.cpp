#include <chrono>
#include <iostream>
#include <stdint.h>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_hil/driver/ChSDLInterface.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include <irrlicht.h>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_hil/ROM/Ch_8DOF_vehicle.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::hil;
using namespace chrono::vehicle;
using namespace chrono::sensor;

int main(int argc, char *argv[]) {

  // Create a ChronoENGINE physical system
  ChSystemSMC sys;

  // Create the terrain
  RigidTerrain terrain(&sys);

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  std::shared_ptr<RigidTerrain::Patch> patch;
  patch = terrain.AddPatch(patch_mat, CSYSNORM, 300, 300);
  patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 10, 10);
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain.Initialize();

  std::string suv_mesh_name = "/vehicles/Nissan_Patrol/FullPatrol.obj";
  auto suv_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  suv_mmesh->LoadWavefrontMesh(
      std::string(STRINGIFY(HIL_DATA_DIR)) + suv_mesh_name, false, true);
  suv_mmesh->RepairDuplicateVertexes(1e-9);

  auto suv_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  suv_trimesh_shape->SetMesh(suv_mmesh);
  suv_trimesh_shape->SetMutable(false);

  auto vehicle_shell = chrono_types::make_shared<ChBodyAuxRef>();
  vehicle_shell->SetCollide(false);
  vehicle_shell->SetPos(ChVector<>(0, 0.0, 0.3));

  ChQuaternion<> initRot = ChQuaternion<>(0, 0, 0, 0);
  vehicle_shell->SetRot(initRot);

  vehicle_shell->SetBodyFixed(true);
  vehicle_shell->AddVisualShape(suv_trimesh_shape);
  sys.Add(vehicle_shell);

  // Create the camera sensor
  auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
  float intensity = 1.2;
  manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  auto cam = chrono_types::make_shared<ChCameraSensor>(
      vehicle_shell, // body camera is attached to
      35,            // update rate in Hz
      chrono::ChFrame<double>(
          ChVector<>(-16, 0.0, 4.0),
          Q_from_Euler123(ChVector<>(0.0, 0.15, 0.0))), // offset pose
      1280,                                             // image width
      720,                                              // image height
      1.608f,
      1); // fov, lag, exposure
  cam->SetName("Camera Sensor");

  cam->PushFilter(
      chrono_types::make_shared<ChFilterVisualize>(1280, 720, "test", false));
  // Provide the host access to the RGBA8 buffer
  cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(cam);

  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  std::string joystick_file =
      (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G27.json");
  SDLDriver.SetJoystickConfigFile(joystick_file);

  // Vehicle parameters JSON file
  std::string vehParamsJSON =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/HMMWV.json";

  // Tire parameters JSON file
  std::string tireParamsJSON =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/TMeasy.json";

  Ch_8DOF_vehicle rom_veh(vehParamsJSON, tireParamsJSON, 0.35);

  // now lets run our simulation
  float time = 0;
  int step_number = 0; // time step counter
  float step_size = rom_veh.GetStepSize();

  ChRealtimeCumulative realtime_timer;

  manager->Update();

  while (true) {

    if (step_number == 0) {
      realtime_timer.Reset();
    }

    // get the controls for this time step
    // Driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_steering = SDLDriver.GetSteering();
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    rom_veh.Advance(time, driver_inputs);

    vehicle_shell->SetPos(rom_veh.GetPos());

    vehicle_shell->SetRot(rom_veh.GetRot());

    // std::cout << "x: " << veh1_st._x << ",y:" << veh1_st._y << std::endl;

    time += step_size;
    step_number += 1;

    sys.DoStepDynamics(step_size);
    manager->Update();

    realtime_timer.Spin(time);
    std::cout << "t:" << time << std::endl;
  }
  return 0;
}