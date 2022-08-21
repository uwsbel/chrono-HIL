// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Simone Benatti, Jason Zhou
// =============================================================================
//
// Iowa Highway Simulation
// This demo includes IG interactive vehicle, autonomous dynamic vehicles and
// autonomous dummy vehicles
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <irrlicht.h>
#include <limits>
#include <time.h>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

//#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_hil/driver/ChCSLDriver.h"
#include "chrono_hil/driver/ChNSF_Drivers.h"
#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#ifdef CHRONO_IRRKLANG
#include "extras/ChCSLSoundEngine.h"
#endif

#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::synchrono;
using namespace chrono::hil;

std::string json_parameters;

struct IG_vehicle {
  std::string vehicle_file;
  std::string powertrain_file;
  std::string tire_file;
  ChVector<> initial_pos;
  ChQuaternion<> initial_rot;
  std::string joystick;
};

struct Terrain {
  RigidTerrain::PatchType terrain_model;
  ChVector<> pos;
  float length;
  float width;
};

struct Asset {
  std::string name;
  ChVector<> pos;
  std::string mesh_file;
  std::string kd_texture;
  std::string roughness_texture;
  std::string normalmap_texture;
  std::string metallic_texture;
  int mutable_ind; // 0 for mutable false. 1 for mutable true
};

// variable initial declaration
IG_vehicle my_ig_vehicle;
Terrain my_terrain;
std::vector<Asset> my_assets;

ChContactMethod contact_method = ChContactMethod::SMC;
ChVector<> trackPoint(0.0, 0.0, 0.5);
float step_size = 1e-3;

void PrintErrorMsg(int num);

void ReadParameterFiles() {
  rapidjson::Document d;
  std::cout << "json: " << json_parameters << std::endl;
  vehicle::ReadFileJSON(json_parameters, d);

  // IG_vehicle specification
  if (d.HasMember("IG_Vehicle")) {
    if (d["IG_Vehicle"].HasMember("vehicle_file")) {
      my_ig_vehicle.vehicle_file = d["IG_Vehicle"]["vehicle_file"].GetString();
    } else {
      PrintErrorMsg(1);
    }

    if (d["IG_Vehicle"].HasMember("powertrain_file")) {
      my_ig_vehicle.powertrain_file =
          d["IG_Vehicle"]["powertrain_file"].GetString();
    } else {
      PrintErrorMsg(1);
    }

    if (d["IG_Vehicle"].HasMember("tire_file")) {
      my_ig_vehicle.tire_file = d["IG_Vehicle"]["tire_file"].GetString();
    } else {
      PrintErrorMsg(1);
    }

    if (d["IG_Vehicle"].HasMember("initial_pos")) {
      auto marr = d["IG_Vehicle"]["initial_pos"].GetArray();
      my_ig_vehicle.initial_pos = ChVector<>(
          marr[0].GetDouble(), marr[1].GetDouble(), marr[2].GetDouble());
    } else {
      my_ig_vehicle.initial_pos = ChVector<>(0.0, 0.0, 0.0);
    }

    if (d["IG_Vehicle"].HasMember("initial_rot")) {
      auto marr = d["IG_Vehicle"]["initial_rot"].GetArray();
      my_ig_vehicle.initial_rot =
          ChQuaternion<>(marr[0].GetDouble(), marr[1].GetDouble(),
                         marr[2].GetDouble(), marr[3].GetDouble());
    } else {
      my_ig_vehicle.initial_rot = ChQuaternion<>(1.0, 0.0, 0.0, 0.0);
    }

    if (d["IG_Vehicle"].HasMember("joystick")) {
      my_ig_vehicle.joystick = d["IG_Vehicle"]["joystick"].GetString();
    }
  } else {
    PrintErrorMsg(0);
  }

  // Terrain specification
  if (d.HasMember("Terrain")) {
    if (d["Terrain"].HasMember("type")) {
      std::string type_string = d["Terrain"]["type"].GetString();
      if (type_string == "BOX") {
        my_terrain.terrain_model = RigidTerrain::PatchType::BOX;
      }
    } else {
      PrintErrorMsg(3);
    }

    if (d["Terrain"].HasMember("pos")) {
      auto marr = d["Terrain"]["pos"].GetArray();
      my_terrain.pos = ChVector<>(marr[0].GetDouble(), marr[1].GetDouble(),
                                  marr[2].GetDouble());
    } else {
      my_terrain.pos = ChVector<>(0.0, 0.0, 0.0);
    }

    if (d["Terrain"].HasMember("length")) {
      my_terrain.length = d["Terrain"]["length"].GetDouble();
    } else {
      my_terrain.length = 300.0;
    }

    if (d["Terrain"].HasMember("width")) {
      my_terrain.width = d["Terrain"]["width"].GetDouble();
    } else {
      my_terrain.width = 300.0;
    }

  } else {
    PrintErrorMsg(2);
  }

  // Assets specification
  int asset_index = 0;
  while (d.HasMember(
      (std::string("Asset") + std::to_string(asset_index)).c_str())) {
    std::string assetname = std::string("Asset") + std::to_string(asset_index);
    Asset temp_asset;
    if (d[assetname.c_str()].HasMember("name")) {
      temp_asset.name = d[assetname.c_str()]["name"].GetString();
    }

    if (d[assetname.c_str()].HasMember("pos")) {
      auto marr = d[assetname.c_str()]["pos"].GetArray();
      temp_asset.pos = ChVector<>(marr[0].GetDouble(), marr[1].GetDouble(),
                                  marr[2].GetDouble());
    }

    if (d[assetname.c_str()].HasMember("mesh")) {
      temp_asset.mesh_file = d[assetname.c_str()]["mesh"].GetString();
    }

    if (d[assetname.c_str()].HasMember("mutable")) {
      temp_asset.mutable_ind = d[assetname.c_str()]["mutable"].GetInt();
    }

    if (d[assetname.c_str()].HasMember("kd_texture")) {
      temp_asset.kd_texture = d[assetname.c_str()]["kd_texture"].GetString();
    }

    if (d[assetname.c_str()].HasMember("roughness_texture")) {
      temp_asset.roughness_texture =
          d[assetname.c_str()]["roughtness_texture"].GetString();
    }

    if (d[assetname.c_str()].HasMember("normalmap_texture")) {
      temp_asset.normalmap_texture =
          d[assetname.c_str()]["normalmap_texture"].GetString();
    }

    if (d[assetname.c_str()].HasMember("metallic_texture")) {
      temp_asset.metallic_texture =
          d[assetname.c_str()]["metallic_texture"].GetString();
    }

    my_assets.push_back(temp_asset);
  }
}

void AddCommandLineOptions(ChCLI &cli) {
  cli.AddOption<std::string>(
      "Simulation", "json_parameters",
      "point to the simulation parameter file which defines environment",
      json_parameters);
}

int main(int argc, char *argv[]) {
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  if (!cli.Parse(argc, argv, true))
    return 0;

  json_parameters = std::string(STRINGIFY(HIL_DATA_DIR)) +
                    cli.GetAsType<std::string>("json_parameters");

  ReadParameterFiles();

  // setup IG_vehicle
  WheeledVehicle vehicle(std::string(STRINGIFY(HIL_DATA_DIR)) +
                             my_ig_vehicle.vehicle_file,
                         ChContactMethod::SMC);
  auto ego_chassis = vehicle.GetChassis();

  vehicle.Initialize(
      ChCoordsys<>(my_ig_vehicle.initial_pos, my_ig_vehicle.initial_rot));
  vehicle.GetChassis()->SetFixed(false);
  auto powertrain = ReadPowertrainJSON(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                       my_ig_vehicle.powertrain_file);
  vehicle.InitializePowertrain(powertrain);

  // Create and initialize the tires
  for (auto &axle : vehicle.GetAxles()) {
    for (auto &wheel : axle->GetWheels()) {
      auto tire = ReadTireJSON(std::string(STRINGIFY(HIL_DATA_DIR)) +
                               my_ig_vehicle.tire_file);
      tire->SetStepsize(step_size);
      vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
    }
  }

  vehicle.SetChassisVisualizationType(VisualizationType::MESH);
  vehicle.SetSuspensionVisualizationType(VisualizationType::MESH);
  vehicle.SetSteeringVisualizationType(VisualizationType::MESH);
  vehicle.SetWheelVisualizationType(VisualizationType::MESH);

  std::cout << "Finished IG_Vehicle Setup" << std::endl;
  // setup terrain

  auto terrain = std::make_shared<RigidTerrain>(vehicle.GetSystem());

  MaterialInfo minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;
  switch (my_terrain.terrain_model) {
  case RigidTerrain::PatchType::BOX:
    patch = terrain->AddPatch(patch_mat, CSYSNORM, my_terrain.length,
                              my_terrain.width);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200,
                      200);
    break;
  }
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain->Initialize();

  std::cout << "Finished Terrain Setup" << std::endl;

  // setup assets

  std::cout << "Finished Asset Setup" << std::endl;

  auto manager =
      chrono_types::make_shared<ChSensorManager>(vehicle.GetSystem());
  float intensity = 2.0;
  manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  // -------------------------------------------------------
  // Create a camera and add it to the sensor manager
  // -------------------------------------------------------
  ChVector<> driver_eye(-6.5, 0.0, 2.5);
  ChQuaternion<> driver_view_direction = Q_from_AngAxis(CH_C_PI / 7, {0, 1, 0});
  auto cam = chrono_types::make_shared<ChCameraSensor>(
      vehicle.GetChassisBody(), // body camera is attached to
      50,                       // update rate in Hz
      chrono::ChFrame<double>(driver_eye, driver_view_direction), // offset pose
      1920,                                                       // image width
      1080, // image height
      1.608f,
      1); // fov, lag, exposure
  cam->SetName("Camera Sensor");

  cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
      1920, 1080, "Driver View", false));

  // add sensor to the manager
  manager->AddSensor(cam);

  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  SDLDriver.SetJoystickConfigFile(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                  my_ig_vehicle.joystick);

  // Simulation Loop
  int step_number = 0;
  while (true) {
    double time = vehicle.GetSystem()->GetChTime();

    // Get driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_steering = -SDLDriver.GetSteering();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    // Update modules (process inputs from other modules)
    int status = SDLDriver.Synchronize();
    if (status == 1) {
      return -1;
    }

    terrain->Synchronize(time);
    vehicle.Synchronize(time, driver_inputs, *terrain);

    // Advance simulation for one timestep for all modules
    // driver.Advance(step_size);
    terrain->Advance(step_size);
    vehicle.Advance(step_size);

    manager->Update();

    // Increment frame number
    step_number++;
  }
}

void PrintErrorMsg(int num) {
  if (num == 0) {
    std::cout << std::endl << "ERROR: missing IG_vehicle field" << std::endl;
  } else if (num == 1) {
    std::cout << std::endl
              << "ERROR: missing IG_vehicle type specification" << std::endl;
  } else if (num == 2) {
    std::cout << std::endl
              << "ERROR: missing terrain specification" << std::endl;
  } else if (num == 3) {
    std::cout << std::endl << "ERROR: missing terrain type" << std::endl;
  }
}
