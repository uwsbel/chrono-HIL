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
// =============================================================================
//
// Basic demonstration of multiple wheeled vehicles in a single simulation using
// the SynChrono wrapper
//
// =============================================================================

#include <chrono>
#include <functional>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"

#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"

#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/controller/SynControllerFunctions.h"
#include "chrono_synchrono/controller/driver/SynMultiPathDriver.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_hil/driver/ChLidarWaypointDriver.h"
#include "chrono_hil/driver/ChSDLInterface.h"

// =============================================================================

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::hil;
// =============================================================================
const double RADS_2_RPM = 30 / CH_C_PI;
const double MS_2_MPH = 2.2369;
// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Type of tire model
TireModelType tire_model = TireModelType::TMEASY;

// Type of vehicle
enum VehicleType { HMMWV, SEDAN, CITYBUS, AUDI, SUV, TRUCK };

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 2e-3;

// Simulation end time
double end_time = 1000;

// How often SynChrono state messages are interchanged
double heartbeat = 2e-2; // 50[Hz]

bool use_fullscreen = false;

ChVector<> simulation_center = {826.734, -37.97, -64.8};

double loading_radius = 1000;

float resolution_x = 1920;
float resolution_y = 1080;
int supersample = 1;
std::string joystick_filename;
int driver_type = 0; // 0 for data driven, 1 for SDL driven, 2 for stream driven
bool render = false;
int refresh_rate = 35;
float cam_offset = 0.54;

int port_id = 6078;

// dashboard
int port_out = 7021;
std::string dash_out_ip = "192.168.0.2";

std::string demo_data_path = std::string(STRINGIFY(HIL_DATA_DIR));

struct PathVehicleSetup {
  VehicleType vehicle_type;
  ChVector<double> pos;
  ChQuaternion<double> rot;
  std::string path_file;
  double lookahead;
  double speed_gain_p;
};

double suv_lookahead = 5.0;
double audi_tight_lookahead = 6.0;
double suv_pgain = .5;
double audi_pgain = .5;

// =============================================================================

// Forward declares for straight forward helper functions

void AddSceneMeshes(ChSystem *chsystem, RigidTerrain *terrain);

void VehicleProcessMessageCallback(
    std::shared_ptr<SynMessage> message, WheeledVehicle &vehicle,
    std::shared_ptr<SynWheeledVehicleAgent> agent,
    std::shared_ptr<ChLidarWaypointDriver> driver);

// =============================================================================

int main(int argc, char *argv[]) {

  // all the demo data will be in user-specified location
  SetChronoDataPath(demo_data_path);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // --------------
  // Create vehicle
  // --------------

  std::string vehicle_filename =
      vehicle::GetDataFile("audi/json/audi_Vehicle.json");
  std::string powertrain_filename =
      vehicle::GetDataFile("audi/json/audi_SimpleMapPowertrain.json");
  std::string tire_filename =
      vehicle::GetDataFile("audi/json/audi_Pac02Tire.json");

  // Initial vehicle location and orientation
  ChVector<> initLoc(925.434, -53.47, -65.2);
  ChQuaternion<> initRot(1, 0, 0, 0);

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
      tire->SetStepsize(step_size / 5.f);
      my_vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
    }
  }

  auto attached_body = std::make_shared<ChBody>();
  my_vehicle.GetSystem()->AddBody(attached_body);
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // --------------
  // Create systems
  // --------------

  RigidTerrain terrain(my_vehicle.GetSystem());
  AddSceneMeshes(my_vehicle.GetSystem(), &terrain);

  ChContactMaterialData minfo; // values from RigidPlane.json
  minfo.mu = 0.9;              // coefficient of friction
  minfo.cr = 0.01;             // coefficient of restitution
  minfo.Y = 2e7;               // Young's modulus
  minfo.nu = 0.3;              // Poisson ratio
  minfo.kn = 2e5;              // normal stiffness
  minfo.gn = 40.0;             // normal viscous damping
  minfo.kt = 2e5;              // tangential stiffness
  minfo.gt = 20.0;             // tangential viscous damping
  auto patch_mat = minfo.CreateMaterial(contact_method);

  ChVector<> normal = ChVector<>({0, 0, 1});
  ChVector<> up = normal.GetNormalized();
  ChVector<> lateral = Vcross(up, ChWorldFrame::Forward());
  lateral.Normalize();
  ChVector<> forward = Vcross(lateral, up);
  ChMatrix33<> rot;
  rot.Set_A_axis(forward, lateral, up);

  auto patch = terrain.AddPatch(
      patch_mat,
      ChCoordsys<>(ChVector<>({0, 0, -65.554}), rot.Get_A_quaternion()),
      10000.0, 10000.0, 2, false, 1, false);
  terrain.Initialize();

  // --------------
  // Create cam
  // --------------
  // add a sensor manager
  auto manager =
      chrono_types::make_shared<ChSensorManager>(my_vehicle.GetSystem());
  Background b;
  b.mode = BackgroundMode::ENVIRONMENT_MAP; // GRADIENT
  b.env_tex = GetChronoDataFile("/Environments/sky_2_4k.hdr");
  manager->scene->SetBackground(b);
  float brightness = 1.5f;
  manager->scene->AddPointLight({0, 0, 10000},
                                {brightness, brightness, brightness}, 100000);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  // camera at driver's eye location for Audi
  auto driver_cam = chrono_types::make_shared<ChCameraSensor>(
      my_vehicle.GetChassisBody(), // body camera is attached to
      25,                          // update rate in Hz
      chrono::ChFrame<double>({-5, .381, 2.04},
                              Q_from_AngAxis(0, {0, 1, 0})), // offset pose
      1280,                                                  // image width
      720,                                                   // image height
      3.14 / 1.5,                                            // fov
      1);

  driver_cam->SetName("DriverCam");
  driver_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
      1280, 720, "Camera1", false));
  driver_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(driver_cam);

  // --------------
  // Create control
  // --------------
  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  joystick_filename = "/joystick/controller_G27.json";

  SDLDriver.SetJoystickConfigFile(demo_data_path + joystick_filename);

  double sim_time = 0.f;
  double last_time = 0.f;
  int step_number = 0;

  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();

  while (true) {

    sim_time = my_vehicle.GetSystem()->GetChTime();

    // Get driver inputs
    DriverInputs driver_inputs;

    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_steering = SDLDriver.GetSteering();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    // Update modules (process inputs from other modules)

    my_vehicle.Synchronize(sim_time, driver_inputs, terrain);
    terrain.Synchronize(sim_time);

    // Advance simulation for one timestep for all modules
    my_vehicle.Advance(step_size);
    terrain.Advance(step_size);

    manager->Update();

    if (SDLDriver.Synchronize() == 1)
      break;

    // Increment frame number
    step_number++;

    // Log clock time
    if (step_number % 500 == 0) {
      std::chrono::high_resolution_clock::time_point end =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> wall_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                    start);

      SynLog() << (wall_time.count()) / (sim_time - last_time) << "\n";
      last_time = sim_time;
      start = std::chrono::high_resolution_clock::now();
    }
  }

  return 0;
}

void AddSceneMeshes(ChSystem *chsystem, RigidTerrain *terrain) {
  // load all meshes in input file, using instancing where possible
  std::string base_path =
      GetChronoDataFile("/Environments/SanFrancisco/components_new/");
  std::string input_file = base_path + "instance_map_03.csv";
  // std::string input_file = base_path + "instance_map_roads_only.csv";

  std::ifstream infile(input_file);
  if (!infile.is_open())
    throw std::runtime_error("Could not open file " + input_file);
  std::string line, col;
  std::vector<std::string> result;

  std::unordered_map<std::string, std::shared_ptr<ChTriangleMeshConnected>>
      mesh_map;

  auto mesh_body = chrono_types::make_shared<ChBody>();
  mesh_body->SetBodyFixed(true);
  mesh_body->SetCollide(false);
  chsystem->Add(mesh_body);

  int meshes_added = 0;
  int mesh_offset = 0;
  int num_meshes = 20000;

  if (infile.good()) {
    int mesh_count = 0;
    int mesh_limit = mesh_offset + num_meshes;

    while (std::getline(infile, line) && mesh_count < mesh_limit) {

      if (mesh_count < mesh_offset) {
        mesh_count++;
      } else {
        mesh_count++;
        result.clear();
        std::stringstream ss(line);
        while (std::getline(ss, col, ',')) {
          result.push_back(col);
        }
        std::string mesh_name = result[0];
        std::string mesh_obj = base_path + result[1] + ".obj";

        // std::cout << mesh_name << std::endl;
        if (mesh_name.find("EmissionOn") ==
            std::string::npos) { // exlude items with
                                 // emission on

          if (true || mesh_name.find("Road") != std::string::npos) {
            ChVector<double> pos = {std::stod(result[2]), std::stod(result[3]),
                                    std::stod(result[4])};

            if ((pos - simulation_center).Length() < loading_radius) {
              // check if mesh is in map
              bool instance_found = false;
              std::shared_ptr<ChTriangleMeshConnected> mmesh;
              if (mesh_map.find(mesh_obj) != mesh_map.end()) {
                mmesh = mesh_map[mesh_obj];
                instance_found = true;
              } else {
                mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
                mmesh->LoadWavefrontMesh(mesh_obj, false, true);
                mesh_map[mesh_obj] = mmesh;
              }

              ChQuaternion<double> rot = {
                  std::stod(result[5]), std::stod(result[6]),
                  std::stod(result[7]), std::stod(result[8])};
              ChVector<double> scale = {std::stod(result[9]),
                                        std::stod(result[10]),
                                        std::stod(result[11])};

              // if not road, only add visualization with new pos,rot,scale
              auto trimesh_shape =
                  chrono_types::make_shared<ChTriangleMeshShape>();
              trimesh_shape->SetMesh(mmesh);
              trimesh_shape->SetName(mesh_name);
              trimesh_shape->SetScale(scale);
              trimesh_shape->SetMutable(false);

              mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(pos, rot));

              meshes_added++;
            }
          }
        }
      }
    }
    std::cout << "Total meshes: " << meshes_added
              << " | Unique meshes: " << mesh_map.size() << std::endl;
  }
}
