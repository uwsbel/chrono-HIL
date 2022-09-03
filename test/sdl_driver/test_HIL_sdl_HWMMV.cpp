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
// Authors: Radu Serban, Justin Madsen, Jason Zhou
// =============================================================================
//
// This test reuses 95% of the code from demo_VEH_HWMMV
// This test implements an independent steering wheel input using SDL library
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/driver/ChSDLInterface.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

#include "chrono/physics/ChBody.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include <chrono>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::hil;
using namespace chrono::sensor;
using namespace chrono::geometry;
using namespace chrono::collision;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(-70, -70, 1.6);
ChQuaternion<> initRot(1, 0, 0, 0);

enum class DriverMode { DEFAULT, RECORD, PLAYBACK };

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

// Model tierods as bodies (true) or as distance constraints (false)
bool use_tierod_bodies = true;

// Type of tire model (RIGID, RIGID_MESH, TMEASY, PACEJKA, LUGRE, FIALA, PAC89,
// PAC02)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
double terrainHeight = 0;     // terrain height (FLAT terrain only)
double terrainLength = 150.0; // size in X direction
double terrainWidth = 150.0;  // size in Y direction

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 1.5e-3;
double tire_step_size = 1e-4;

// Simulation end time
double t_end = 1000;

// Irrlicht Rendering Window Size
int image_width = 1920;
int image_height = 1080;
int refresh_rate = 30;
int supersample = 1;

// Joystick Configuration File
std::string joystick_filename;

// =============================================================================
void AddObstacle1(RigidTerrain &terrain,
                  std::shared_ptr<RigidTerrain::Patch> patch);
void AddObstacle2(RigidTerrain &terrain,
                  std::shared_ptr<RigidTerrain::Patch> patch);
void AddObstacle3(RigidTerrain &terrain,
                  std::shared_ptr<RigidTerrain::Patch> patch);
// =============================================================================
void AddCommandLineOptions(ChCLI &cli) {
  cli.AddOption<std::string>("Simulation", "joystick_filename",
                             "Joystick config JSON file", joystick_filename);
  cli.AddOption<int>("Simulation", "image_width", "x resolution",
                     std::to_string(image_width));
  cli.AddOption<int>("Simulation", "image_height", "y resolution",
                     std::to_string(image_height));
  cli.AddOption<int>("Simulation", "refresh_rate",
                     "Chrono Sensor Refresh Rate - e.g.25Hz",
                     std::to_string(refresh_rate));
  cli.AddOption<int>("Simulation", "supersample", "supersample factor",
                     std::to_string(supersample));
}
// =============================================================================

int main(int argc, char *argv[]) {
  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: "
           << CHRONO_VERSION << "\n\n";

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  if (!cli.Parse(argc, argv, true))
    return 0;

  // parse from cli
  image_width = cli.GetAsType<int>("image_width");
  image_height = cli.GetAsType<int>("image_height");
  joystick_filename = std::string(STRINGIFY(HIL_DATA_DIR)) +
                      cli.GetAsType<std::string>("joystick_filename");
  refresh_rate = cli.GetAsType<int>("refresh_rate");
  supersample = cli.GetAsType<int>("supersample");

  // --------------
  // Create systems
  // --------------

  // Create the HMMWV vehicle, set parameters, and initialize
  HMMWV_Full my_hmmwv;
  my_hmmwv.SetContactMethod(contact_method);
  my_hmmwv.SetChassisCollisionType(chassis_collision_type);
  my_hmmwv.SetChassisFixed(false);
  my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  my_hmmwv.SetPowertrainType(powertrain_model);
  my_hmmwv.SetDriveType(drive_type);
  my_hmmwv.UseTierodBodies(use_tierod_bodies);
  my_hmmwv.SetSteeringType(steering_type);
  my_hmmwv.SetTireType(tire_model);
  my_hmmwv.SetTireStepSize(tire_step_size);
  my_hmmwv.Initialize();

  auto attached_body = std::make_shared<ChBody>();
  my_hmmwv.GetSystem()->AddBody(attached_body);
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  if (tire_model == TireModelType::RIGID_MESH)
    tire_vis_type = VisualizationType::MESH;

  my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
  my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
  my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
  my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
  my_hmmwv.SetTireVisualizationType(tire_vis_type);

  // Create the terrain
  RigidTerrain terrain(my_hmmwv.GetSystem());

  MaterialInfo minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;
  patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
  patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 10, 10);
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain.Initialize();

  // Optionally, attach additional visual assets to ground.
  // Note: this must be done after initializing the terrain (so that its visual
  // model is created).
  if (patch->GetGroundBody()->GetVisualModel()) {
    for (int i = 0; i < 8; i++) {
      for (int j = 0; j < 2; j++) {
        auto trimesh =
            geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
                STRINGIFY(HIL_DATA_DIR) +
                    std::string("/Environments/HWMMV_test/trees/tree_01.obj"),
                true, true);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName("Trees");
        trimesh_shape->SetMutable(false);
        ChVector<> tree_pos(-70.0 + i * 20.0, -25 + j * 75.0, 0.0);
        patch->GetGroundBody()->GetVisualModel()->AddShape(
            trimesh_shape, ChFrame<>(tree_pos, Q_from_AngZ(CH_C_PI_2)));
      }
    }
  }

  // Add obstacles
  AddObstacle1(terrain, patch);
  AddObstacle2(terrain, patch);
  AddObstacle3(terrain, patch);

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
      refresh_rate,  // update rate in Hz
      chrono::ChFrame<double>(
          ChVector<>(-17.0, 0.0, 4.0),
          Q_from_Euler123(ChVector<>(0.0, 0.15, 0.0))), // offset pose
      image_width,                                      // image width
      image_height,                                     // image height
      1.608f,
      supersample); // fov, lag, exposure
  cam->SetName("Camera Sensor");

  cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
      image_width, image_height, "hwwmv", false));
  // Provide the host access to the RGBA8 buffer
  cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(cam);

  // ------------------------
  // Create the driver system
  // ------------------------

  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  SDLDriver.SetJoystickConfigFile(joystick_filename);

  // ---------------
  // Simulation loop
  // ---------------

  my_hmmwv.GetVehicle().LogSubsystemTypes();

  // Initialize simulation frame counters
  int step_number = 0;

  my_hmmwv.GetVehicle().EnableRealtime(false);

  float last_lag = 0.0;
  auto start = std::chrono::high_resolution_clock::now();

  while (true) {
    double time = my_hmmwv.GetSystem()->GetChTime();

    ChVector<> pos = my_hmmwv.GetChassis()->GetPos();
    ChQuaternion<> rot = my_hmmwv.GetChassis()->GetRot();

    auto euler_rot = Q_to_Euler123(rot);
    euler_rot.y() = 0.0;
    auto y_0_rot = Q_from_Euler123(euler_rot);

    attached_body->SetPos(pos);
    attached_body->SetRot(y_0_rot);

    manager->Update();

    // Driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_steering = SDLDriver.GetSteering();
    driver_inputs.m_braking = SDLDriver.GetBraking();
    // std::cout << "throttle: " << driver_inputs.m_throttle
    //           << ", brake: " << driver_inputs.m_braking
    //           << ", steer:" << driver_inputs.m_steering << std::endl;

    // Update modules (process inputs from other modules)
    terrain.Synchronize(time);
    my_hmmwv.Synchronize(time, driver_inputs, terrain);

    // Advance simulation for one timestep for all modules
    terrain.Advance(step_size);
    my_hmmwv.Advance(step_size);

    if (step_number == 0) {
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> diff = end - start;
      last_lag = diff.count();
    }

    if (step_number % 1000 == 0 && step_number != 0) {
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> diff = end - start;
      std::cout << "RTF: " << (diff.count() - last_lag) / (1000 * step_size)
                << std::endl;
      last_lag = diff.count();
    }

    // Increment frame number
    step_number++;

    if (SDLDriver.Synchronize() == 1) {
      return 0;
    }
  }

  return 0;
}

void AddObstacle1(RigidTerrain &terrain,
                  std::shared_ptr<RigidTerrain::Patch> patch) {

  for (int i = 0; i < 12; i++) {
    ChVector<> ob_pos(-50 + 5 * i, -60, -0.85);
    auto patch1_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch1_mat->SetFriction(0.98f);
    patch1_mat->SetRestitution(0.002f);
    auto patch1 = terrain.AddPatch(
        patch1_mat, ChCoordsys<>(ob_pos, Q_from_AngZ(CH_C_PI_2)),
        STRINGIFY(HIL_DATA_DIR) +
            std::string("/Environments/HWMMV_test/cyl_bump/cyl_bump.obj"));

    patch1->GetGroundBody()->GetCollisionModel()->ClearModel();
    patch1->GetGroundBody()->GetCollisionModel()->AddCylinder(
        patch1_mat, 1, 1, 5, ChVector<>(0, 0, 0), Q_from_AngZ(CH_C_PI_2));
    patch1->GetGroundBody()->GetCollisionModel()->BuildModel();

    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
        STRINGIFY(HIL_DATA_DIR) +
            std::string("/Environments/HWMMV_test/cyl_bump/cyl_bump.obj"),
        true, true);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("cyl_bump");
    trimesh_shape->SetMutable(false);
    trimesh_shape->SetTexture(
        vehicle::GetDataFile("terrain/textures/concrete.jpg"));
    patch->GetGroundBody()->GetVisualModel()->AddShape(
        trimesh_shape, ChFrame<>(ob_pos, Q_from_AngZ(CH_C_PI_2)));
  }

  for (int i = 0; i < 5; i++) {
    ChVector<> ob_pos(15 + 8 * i, -57.2, -0.85);
    auto patch1_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch1_mat->SetFriction(0.98f);
    patch1_mat->SetRestitution(0.002f);
    auto patch1 = terrain.AddPatch(
        patch1_mat, ChCoordsys<>(ob_pos, Q_from_AngZ(CH_C_PI_2)),
        STRINGIFY(HIL_DATA_DIR) +
            std::string("/Environments/HWMMV_test/cyl_bump/cyl_bump.obj"));

    patch1->GetGroundBody()->GetCollisionModel()->ClearModel();
    patch1->GetGroundBody()->GetCollisionModel()->AddCylinder(
        patch1_mat, 1, 1, 2.5, ChVector<>(0, 0, 0), Q_from_AngZ(CH_C_PI_2));
    patch1->GetGroundBody()->GetCollisionModel()->BuildModel();

    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
        STRINGIFY(HIL_DATA_DIR) +
            std::string("/Environments/HWMMV_test/cyl_bump/cyl_bump_half.obj"),
        true, true);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("cyl_bump");
    trimesh_shape->SetMutable(false);
    trimesh_shape->SetTexture(
        vehicle::GetDataFile("terrain/textures/concrete.jpg"));
    patch->GetGroundBody()->GetVisualModel()->AddShape(
        trimesh_shape, ChFrame<>(ob_pos, Q_from_AngZ(CH_C_PI_2)));
  }

  for (int i = 0; i < 5; i++) {
    ChVector<> ob_pos(19 + 8 * i, -62.8, -0.85);
    auto patch1_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch1_mat->SetFriction(0.98f);
    patch1_mat->SetRestitution(0.002f);
    auto patch1 = terrain.AddPatch(
        patch1_mat, ChCoordsys<>(ob_pos, Q_from_AngZ(CH_C_PI_2)),
        STRINGIFY(HIL_DATA_DIR) +
            std::string("/Environments/HWMMV_test/cyl_bump/cyl_bump.obj"));

    patch1->GetGroundBody()->GetCollisionModel()->ClearModel();
    patch1->GetGroundBody()->GetCollisionModel()->AddCylinder(
        patch1_mat, 1, 1, 2.5, ChVector<>(0, 0, 0), Q_from_AngZ(CH_C_PI_2));
    patch1->GetGroundBody()->GetCollisionModel()->BuildModel();

    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
        STRINGIFY(HIL_DATA_DIR) +
            std::string("/Environments/HWMMV_test/cyl_bump/cyl_bump_half.obj"),
        true, true);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("cyl_bump");
    trimesh_shape->SetMutable(false);
    trimesh_shape->SetTexture(
        vehicle::GetDataFile("terrain/textures/concrete.jpg"));
    patch->GetGroundBody()->GetVisualModel()->AddShape(
        trimesh_shape, ChFrame<>(ob_pos, Q_from_AngZ(CH_C_PI_2)));
  }
}

void AddObstacle2(RigidTerrain &terrain,
                  std::shared_ptr<RigidTerrain::Patch> patch) {
  for (int i = 0; i < 8; i++) {
    ChVector<> ob_pos(50 - 15 * i, 0.0, 0.0);
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
        STRINGIFY(HIL_DATA_DIR) +
            std::string("/Environments/HWMMV_test/cone/cone.obj"),
        true, true);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("cone");
    trimesh_shape->SetMutable(false);
    patch->GetGroundBody()->GetVisualModel()->AddShape(
        trimesh_shape, ChFrame<>(ob_pos, Q_from_AngZ(CH_C_PI_2)));
  }
}

void AddObstacle3(RigidTerrain &terrain,
                  std::shared_ptr<RigidTerrain::Patch> patch) {
  ChVector<> ob_pos(20, 60, 0.0);

  auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
      STRINGIFY(HIL_DATA_DIR) +
          std::string("/Environments/HWMMV_test/hill/hill_obs.obj"),
      true, true);
  auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  trimesh_shape->SetMesh(trimesh);
  trimesh_shape->SetName("hill");
  trimesh_shape->SetMutable(false);
  trimesh_shape->SetTexture(
      vehicle::GetDataFile("terrain/textures/concrete.jpg"));
  patch->GetGroundBody()->GetVisualModel()->AddShape(
      trimesh_shape, ChFrame<>(ob_pos, Q_from_AngZ(CH_C_PI_2)));

  auto patch1_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
  patch1_mat->SetFriction(0.98f);
  patch1_mat->SetRestitution(0.002f);
  auto patch1 = terrain.AddPatch(
      patch1_mat, ChCoordsys<>(ob_pos, Q_from_AngZ(CH_C_PI_2)),
      STRINGIFY(HIL_DATA_DIR) +
          std::string("/Environments/HWMMV_test/hill/hill_obs.obj"));

  patch1->GetGroundBody()->GetCollisionModel()->ClearModel();

  std::string lugged_file(
      STRINGIFY(HIL_DATA_DIR) +
      std::string("/Environments/HWMMV_test/hill/hill_obs.obj"));
  geometry::ChTriangleMeshConnected lugged_mesh;
  ChConvexDecompositionHACDv2 lugged_convex;
  utils::LoadConvexMesh(lugged_file, lugged_mesh, lugged_convex);
  int num_hulls = lugged_convex.GetHullCount();

  for (int ihull = 0; ihull < num_hulls; ihull++) {
    std::vector<ChVector<>> convexhull;
    lugged_convex.GetConvexHullResult(ihull, convexhull);
    patch1->GetGroundBody()->GetCollisionModel()->AddConvexHull(
        patch1_mat, convexhull, VNULL, QUNIT);
  }
  patch1->GetGroundBody()->GetCollisionModel()->BuildModel();
}
