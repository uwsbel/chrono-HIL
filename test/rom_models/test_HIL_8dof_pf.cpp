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

#include "chrono_hil/ROM/driver/Ch_ROM_PathFollowerDriver.h"
#include "chrono_hil/ROM/veh/Ch_8DOF_vehicle.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono/core/ChBezierCurve.h"

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

  std::string hmmwv_rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/patrol/patrol_rom.json";

  std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
      chrono_types::make_shared<Ch_8DOF_vehicle>(hmmwv_rom_json, 0.45);

  rom_veh->SetInitPos(ChVector<>(50.0, 0.0, 0.0));
  rom_veh->Initialize(sys);

  std::shared_ptr<ChBezierCurve> path = ChBezierCurve::read(
      STRINGIFY(HIL_DATA_DIR) +
          std::string("/ring/terrain0103/ring50_closed.txt"),
      true);

  Ch_ROM_PathFollowerDriver driver(rom_veh, path, ChVector<>(0, 0, 1), 10.0,
                                   10.0, 0.1, 0.0, 0.0, 0.3, 0.0, 0.0);

  auto attached_body = std::make_shared<ChBody>();
  sys.AddBody(attached_body);
  attached_body->SetPos(ChVector<>(0.0, 0.0, 0.0));
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // now lets run our simulation
  float time = 0;
  int step_number = 0; // time step counter
  float step_size = rom_veh->GetStepSize();

  // Create the camera sensor
  auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
  float intensity = 1.2;
  manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);
  /*
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        rom_veh->GetChassisBody(), // body camera is attached to
        35,                        // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(0.0, -8.0, 3.0),
            Q_from_Euler123(ChVector<>(0.0, 0.15, C_PI / 2))), // offset pose
        1280,                                                  // image width
        720,                                                   // image height
        1.608f,
        1); // fov, lag, exposure
    cam->SetName("Camera Sensor");

    cam->PushFilter(
        chrono_types::make_shared<ChFilterVisualize>(1280, 720, "test", false));
    // Provide the host access to the RGBA8 buffer
    // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);
  */
  auto cam2 = chrono_types::make_shared<ChCameraSensor>(
      attached_body, // body camera is attached to
      35,            // update rate in Hz
      chrono::ChFrame<double>(
          ChVector<>(0.0, 0.0, 100.0),
          Q_from_Euler123(ChVector<>(0.0, C_PI / 2, 0.0))), // offset pose
      1280,                                                 // image width
      720,                                                  // image
      1.608f, 1); // fov, lag, exposure cam2->SetName("Camera Sensor");

  cam2->PushFilter(
      chrono_types::make_shared<ChFilterVisualize>(1280, 720, "test", false));
  // Provide the host access to the RGBA8 buffer
  // cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(cam2);

  manager->Update();

  ChRealtimeCumulative realtime_timer;
  while (true) {

    if (step_number == 0) {
      realtime_timer.Reset();
    }

    // get the controls for this time step
    // Driver inputs
    DriverInputs driver_inputs;
    driver.Advance(step_size);
    driver_inputs = driver.GetDriverInput();
    rom_veh->Advance(time, driver_inputs);

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