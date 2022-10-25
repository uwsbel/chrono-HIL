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

// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

// =============================================================================

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::hil;

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
enum VehicleType { HMMWV, SEDAN };

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 1e-3;

// Simulation end time
double end_time = 1000;

// How often SynChrono state messages are interchanged
double heartbeat = 5e-3; // 100[Hz]

bool use_fullscreen = false;

ChVector<> simulation_center = {826.734, -37.97, -64.8};

double loading_radius = 1000;

float resolution_x = 1920;
float resolution_y = 1080;
int supersample = 1;
std::string joystick_filename;
bool use_data_driver = false;
bool render = false;

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

// starting locations and paths
std::vector<PathVehicleSetup> demo_config = {
    {HMMWV,
     {925.434, -150.87, -65.2},
     Q_from_AngZ(3.14 / 2),
     "/paths/2.txt",
     8.0,
     0.1}, // ego vehicle

    {SEDAN,
     {925.434, -53.47, -65.2},
     Q_from_AngZ(3.14 / 2),
     "/paths/2.txt",
     8.0,
     0.1},
    {HMMWV,
     {925.434, 0.47, -65.2},
     Q_from_AngZ(3.14 / 2),
     "/paths/2.txt",
     audi_tight_lookahead,
     audi_pgain},
    /*
   {VAN,
    {925.434, 50.47, -64.8},
    Q_from_AngZ(3.14 / 2),
    "/paths/2.txt",
    8.0,
    1.0},
   {SUV,
    {925.434, 75.47, -64.8},
    Q_from_AngZ(3.14 / 2),
    "/paths/2.txt",
    suv_lookahead,
    suv_pgain},
   {AUDI,
    {903.134, 149.13, -64.8},
    Q_from_AngZ(3.14),
    "/paths/2.txt",
    audi_tight_lookahead,
    audi_pgain},
   {AUDI,
    {825.134, 149.13, -64.8},
    Q_from_AngZ(3.14),
    "/paths/2.txt",
    audi_tight_lookahead,
    audi_pgain},
   {SUV,
    {751.234, 148.93, -64.8},
    Q_from_AngZ(3.14),
    "/paths/2.txt",
    suv_lookahead,
    suv_pgain},
   {CITYBUS,
    {727.834, 124.13, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/2.txt",
    5.0,
    1.0},
   {SUV,
    {727.834, 85.13, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/2.txt",
    suv_lookahead,
    suv_pgain},
   {AUDI,
    {727.834, 40.13, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/2.txt",
    audi_tight_lookahead,
    audi_pgain},
   {SUV,
    {727.834, -34.27, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/2.txt",
    suv_lookahead,
    suv_pgain},
   {AUDI,
    {727.834, -100.27, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/2.txt",
    audi_tight_lookahead,
    audi_pgain},
   {AUDI,
    {727.834, -212.97, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/2.txt",
    audi_tight_lookahead,
    audi_pgain},
   {VAN, {748.234, -225.07, -64.8}, Q_from_AngZ(0), "/paths/2.txt", 8.0, 1.0},
   {AUDI,
    {855.934, -222.77, -64.8},
    Q_from_AngZ(0),
    "/paths/2.txt",
    audi_tight_lookahead,
    audi_pgain},
   {CITYBUS,
    {925.634, -214.17, -64.8},
    Q_from_AngZ(3.14 / 2),
    "/paths/2.txt",
    5.0,
    1.0},

   {AUDI,
    {867.634, 140.83, -64.8},
    Q_from_AngZ(0),
    "/paths/3.txt",
    audi_tight_lookahead,
    audi_pgain},
   {AUDI,
    {847.634, 140.83, -64.8},
    Q_from_AngZ(0),
    "/paths/3.txt",
    audi_tight_lookahead,
    audi_pgain},
   {AUDI,
    {917.234, 116.63, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/3.txt",
    audi_tight_lookahead,
    audi_pgain},
   {SUV,
    {917.234, 60.63, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/3.txt",
    suv_lookahead,
    suv_pgain},
   {SUV,
    {917.234, -10.63, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/3.txt",
    suv_lookahead,
    suv_pgain},
   {AUDI,
    {917.334, -95.67, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/3.txt",
    audi_tight_lookahead,
    audi_pgain},
   {SUV,
    {892.334, -120.17, -64.8},
    Q_from_AngZ(3.14),
    "/paths/3.txt",
    suv_lookahead,
    suv_pgain},
   {SUV,
    {850.334, -120.17, -64.8},
    Q_from_AngZ(3.14),
    "/paths/3.txt",
    suv_lookahead,
    suv_pgain},
   {AUDI,
    {752.934, -119.47, -64.8},
    Q_from_AngZ(3.14),
    "/paths/3.txt",
    audi_tight_lookahead,
    audi_pgain},
   {SUV,
    {735.734, -102.97, -64.8},
    Q_from_AngZ(3.14 / 2),
    "/paths/3.txt",
    suv_lookahead,
    suv_pgain},
   {AUDI,
    {735.734, -75.97, -64.8},
    Q_from_AngZ(3.14 / 2),
    "/paths/3.txt",
    audi_tight_lookahead,
    audi_pgain},
   {AUDI,
    {735.734, 1.43, -64.8},
    Q_from_AngZ(3.14 / 2),
    "/paths/3.txt",
    audi_tight_lookahead,
    audi_pgain},
   {AUDI,
    {735.734, 123.63, -64.8},
    Q_from_AngZ(3.14 / 2),
    "/paths/3.txt",
    audi_tight_lookahead,
    audi_pgain},
   {SUV,
    {755.634, 140.93, -64.8},
    Q_from_AngZ(0),
    "/paths/3.txt",
    suv_lookahead,
    suv_pgain},
   {SUV,
    {785.634, 140.93, -64.8},
    Q_from_AngZ(0),
    "/paths/3.txt",
    suv_lookahead,
    suv_pgain},

   {AUDI,
    {845.534, -131.97, -64.8},
    Q_from_AngZ(3.14),
    "/paths/4.txt",
    audi_tight_lookahead,
    audi_pgain},
   {VAN,
    {763.334, -131.37, -64.8},
    Q_from_AngZ(3.14),
    "/paths/4.txt",
    8.0,
    1.0},
   {SUV,
    {727.834, -158.07, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/4.txt",
    suv_lookahead,
    suv_pgain},
   {SUV,
    {727.834, -203.57, -64.8},
    Q_from_AngZ(-3.14 / 2),
    "/paths/4.txt",
    suv_lookahead,
    suv_pgain},
   {AUDI,
    {759.734, -225.07, -64.8},
    Q_from_AngZ(0),
    "/paths/4.txt",
    audi_tight_lookahead,
    audi_pgain},
   {SUV,
    {897.934, -223.27, -64.8},
    Q_from_AngZ(0),
    "/paths/4.txt",
    suv_lookahead,
    suv_pgain},
   {AUDI,
    {925.434, -199.77, -64.8},
    Q_from_AngZ(3.14 / 2),
    "/paths/4.txt",
    audi_tight_lookahead,
    audi_pgain},
   {AUDI,
    {897.434, -132.07, -64.8},
    Q_from_AngZ(3.14),
    "/paths/4.txt",
    audi_tight_lookahead,
    audi_pgain}
    */

};

// =============================================================================

// Forward declares for straight forward helper functions
void LogCopyright(bool show);
void AddCommandLineOptions(ChCLI &cli);
void GetVehicleModelFiles(VehicleType type, std::string &vehicle,
                          std::string &powertrain, std::string &tire,
                          std::string &zombie, ChVector<> &lidar_pos,
                          double &cam_distance);

void AddSceneMeshes(ChSystem *chsystem, RigidTerrain *terrain);

// =============================================================================

int main(int argc, char *argv[]) {
  // -----------------------------------------------------
  // CLI SETUP - Get most parameters from the command line
  // -----------------------------------------------------

  ChCLI cli(argv[0]);

  AddCommandLineOptions(cli);
  if (!cli.Parse(argc, argv, true))
    return 0;

  const int node_id = cli.GetAsType<int>("node_id");
  const int num_nodes = cli.GetAsType<int>("num_nodes");
  const std::vector<std::string> ip_list =
      cli.GetAsType<std::vector<std::string>>("ip");

  // -----------------------
  // Create SynChronoManager
  // -----------------------

  // Use UDP4
  DomainParticipantQos qos;
  qos.name("/syn/node/" + std::to_string(node_id) + ".0");
  qos.transport().user_transports.push_back(
      std::make_shared<UDPv4TransportDescriptor>());

  qos.transport().use_builtin_transports = false;
  qos.wire_protocol().builtin.avoid_builtin_multicast = false;

  // Set the initialPeersList
  for (const auto &ip : ip_list) {
    Locator_t locator;
    locator.kind = LOCATOR_KIND_UDPv4;
    IPLocator::setIPv4(locator, ip);
    qos.wire_protocol().builtin.initialPeersList.push_back(locator);
  }
  auto communicator = chrono_types::make_shared<SynDDSCommunicator>(qos);
  SynChronoManager syn_manager(node_id, num_nodes, communicator);

  // Change SynChronoManager settings
  syn_manager.SetHeartbeat(heartbeat);

  // all the demo data will be in user-specified location
  SetChronoDataPath(demo_data_path);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));
  synchrono::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // Normal simulation options
  step_size = cli.GetAsType<double>("step_size");
  end_time = cli.GetAsType<double>("end_time");
  heartbeat = cli.GetAsType<double>("heartbeat");
  use_fullscreen = cli.GetAsType<bool>("fullscreen");
  bool record_inputs = cli.GetAsType<bool>("record");
  bool replay_inputs = cli.GetAsType<bool>("replay");
  loading_radius = cli.GetAsType<double>("load_radius");
  joystick_filename = std::string(STRINGIFY(HIL_DATA_DIR)) +
                      cli.GetAsType<std::string>("joystick_filename");
  resolution_x = cli.GetAsType<float>("resolution_x");
  resolution_y = cli.GetAsType<float>("resolution_y");
  supersample = cli.GetAsType<int>("supersample_rate");
  render = cli.GetAsType<bool>("render");
  use_data_driver = cli.GetAsType<bool>("data_driver");

  // Change SynChronoManager settings
  syn_manager.SetHeartbeat(heartbeat);

  // --------------
  // Create systems
  // --------------

  // Adjust position of each vehicle so they aren't on top of each other

  // Get the vehicle JSON filenames
  double cam_distance;
  std::string vehicle_filename, powertrain_filename, tire_filename,
      zombie_filename;
  ChVector<> lidar_pos;

  GetVehicleModelFiles(demo_config[node_id].vehicle_type, vehicle_filename,
                       powertrain_filename, tire_filename, zombie_filename,
                       lidar_pos, cam_distance);

  // Create the vehicle, set parameters, and initialize
  WheeledVehicle vehicle(vehicle_filename, contact_method);
  if (node_id < demo_config.size()) {
    vehicle.Initialize(
        ChCoordsys<>(demo_config[node_id].pos, demo_config[node_id].rot));
  } else {
    vehicle.Initialize(ChCoordsys<>(demo_config[0].pos, demo_config[0].rot));
  }

  vehicle.GetChassis()->SetFixed(false);
  vehicle.SetChassisVisualizationType(chassis_vis_type);
  vehicle.SetSuspensionVisualizationType(suspension_vis_type);
  vehicle.SetSteeringVisualizationType(steering_vis_type);
  vehicle.SetWheelVisualizationType(wheel_vis_type);

  // Create and initialize the powertrain system
  auto powertrain = ReadPowertrainJSON(powertrain_filename);
  vehicle.InitializePowertrain(powertrain);

  // Create and initialize the tires
  for (auto &axle : vehicle.GetAxles()) {
    for (auto &wheel : axle->GetWheels()) {
      auto tire = ReadTireJSON(tire_filename);
      vehicle.InitializeTire(tire, wheel, tire_vis_type);
    }
  }

  // Add vehicle as an agent and initialize SynChronoManager
  auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(
      &vehicle, zombie_filename);
  syn_manager.AddAgent(agent);
  syn_manager.Initialize(vehicle.GetSystem());

  ChRealtimeCumulative realtime_timer;

  RigidTerrain terrain(vehicle.GetSystem());
  AddSceneMeshes(vehicle.GetSystem(), &terrain);

  MaterialInfo minfo; // values from RigidPlane.json
  minfo.mu = 0.9;     // coefficient of friction
  minfo.cr = 0.01;    // coefficient of restitution
  minfo.Y = 2e7;      // Young's modulus
  minfo.nu = 0.3;     // Poisson ratio
  minfo.kn = 2e5;     // normal stiffness
  minfo.gn = 40.0;    // normal viscous damping
  minfo.kt = 2e5;     // tangential stiffness
  minfo.gt = 20.0;    // tangential viscous damping
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

  std::shared_ptr<ChLidarSensor> lidar;
  std::shared_ptr<ChCameraSensor> camera;
  std::shared_ptr<ChSensorManager> manager;

  if (render) {
    // add a sensor manager
    manager = chrono_types::make_shared<ChSensorManager>(vehicle.GetSystem());
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

    const int image_width = resolution_x;
    const int image_height = resolution_y;

    // camera at driver's eye location for Audi
    auto driver_cam = chrono_types::make_shared<ChCameraSensor>(
        vehicle.GetChassisBody(), // body camera is attached to
        30,                       // update rate in Hz
        chrono::ChFrame<double>({0.54, .381, 1.04},
                                Q_from_AngAxis(0, {0, 1, 0})), // offset pose
        image_width,                                           // image width
        image_height,                                          // image height
        3.14 / 1.5,                                            // fov
        supersample);

    driver_cam->SetName("DriverCam");
    driver_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
        image_width, image_height, "Camera1", use_fullscreen));
    driver_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(driver_cam);
  }

  std::string driver_file = "driver_inputs.txt";
  utils::CSV_writer driver_csv(" ");

  std::shared_ptr<ChDriver> driver;
  ChSDLInterface SDLDriver;

  if (!use_data_driver) {
    // Create the interactive driver system
    SDLDriver.Initialize();
    SDLDriver.SetJoystickConfigFile(joystick_filename);
  } else {
    auto path =
        ChBezierCurve::read(GetChronoDataFile(demo_config[node_id].path_file));
    double target_speed = 10;
    bool isPathClosed = true;
    double following_time = 40.0;
    double following_distance = 10;
    double current_distance = 100;

    auto acc_driver = chrono_types::make_shared<ChPathFollowerACCDriver>(
        vehicle, path, "Highway", target_speed, following_time,
        following_distance, current_distance, isPathClosed);
    acc_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
    acc_driver->GetSteeringController().SetGains(0.5, 0.02, 0.0);
    acc_driver->GetSteeringController().SetLookAheadDistance(10);

    driver = acc_driver;
  }

  // ---------------
  // Simulation loop
  // ---------------

  // Initialize simulation frame counters
  int step_number = 0;

  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double last_time = 0;

  float orbit_radius = 10.f;
  float orbit_rate = .25;
  double time = 0;

  while (syn_manager.IsOk()) {

    if (step_number == 1) {
      realtime_timer.Reset();
    }

    time = vehicle.GetSystem()->GetChTime();

    // Get driver inputs
    DriverInputs driver_inputs;

    if (!use_data_driver) {
      if (step_number % 50 == 0) {
        // Create the interactive driver system
        driver_inputs.m_throttle = SDLDriver.GetThrottle();
        driver_inputs.m_steering = SDLDriver.GetSteering();
        driver_inputs.m_braking = SDLDriver.GetBraking();
      }

    } else {
      driver_inputs = driver->GetInputs();
    }

    // Update modules (process inputs from other modules)
    syn_manager.Synchronize(time); // Synchronize between nodes
    if (use_data_driver)
      driver->Synchronize(time);
    vehicle.Synchronize(time, driver_inputs, terrain);
    terrain.Synchronize(time);

    // Advance simulation for one timestep for all modules
    if (use_data_driver)
      driver->Advance(step_size);
    vehicle.Advance(step_size);
    terrain.Advance(step_size);

    if (render) {
      manager->Update();
    }

    if (!use_data_driver) {
      if (SDLDriver.Synchronize() == 1)
        break;
    }

    // Increment frame number
    step_number++;

    if (step_number % 10 == 0) {
      realtime_timer.Spin(time);
    }

    // Log clock time
    if (step_number % 500 == 0) {
      std::chrono::high_resolution_clock::time_point end =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> wall_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                    start);

      SynLog() << (wall_time.count()) / (time - last_time) << "\n";
      last_time = time;
      start = std::chrono::high_resolution_clock::now();
    }
  }

  // Properly shuts down other ranks when one rank ends early
  syn_manager.QuitSimulation();

  return 0;
}

void LogCopyright(bool show) {
  if (!show)
    return;

  SynLog() << "Copyright (c) 2020 projectchrono.org\n";
  SynLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";
}

void AddCommandLineOptions(ChCLI &cli) {
  // Standard demo options
  cli.AddOption<double>("Simulation", "s,step_size", "Step size",
                        std::to_string(step_size));
  cli.AddOption<double>("Simulation", "e,end_time", "End time",
                        std::to_string(end_time));
  cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat",
                        std::to_string(heartbeat));
  cli.AddOption<float>("Simulation", "x,resolution_x", "Resolution x",
                       std::to_string(resolution_x));
  cli.AddOption<float>("Simulation", "y,resolution_y", "Resolution y",
                       std::to_string(resolution_y));
  cli.AddOption<int>("Simulation", "r,supersample_rate", "Supersample Rate",
                     std::to_string(supersample));
  cli.AddOption<bool>("Simulation", "render", "Render rank",
                      std::to_string(render));
  // mesh loading options
  cli.AddOption<double>("Simulation", "load_radius",
                        "Radius around simulation center to load meshes",
                        std::to_string(loading_radius));

  // options for human driver
  cli.AddOption<bool>("Simulation", "fullscreen",
                      "Use full screen camera display",
                      std::to_string(use_fullscreen));
  cli.AddOption<bool>("Simulation", "record",
                      "Record human driver inputs to file", "false");
  cli.AddOption<bool>("Simulation", "replay",
                      "Replay human driver inputs from file", "false");

  // DDS Specific
  cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
  cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
  cli.AddOption<std::vector<std::string>>(
      "DDS", "ip", "IP Addresses for initialPeersList", "127.0.0.1");

  cli.AddOption<std::string>("Simulation", "joystick_filename",
                             "Joystick config JSON file", joystick_filename);
  cli.AddOption<bool>("Simulation", "data_driver", "whether to use data driver",
                      std::to_string(use_data_driver));
}

void GetVehicleModelFiles(VehicleType type, std::string &vehicle,
                          std::string &powertrain, std::string &tire,
                          std::string &zombie, ChVector<> &lidar_pos,
                          double &cam_distance) {
  switch (type) {
  case VehicleType::HMMWV:
    vehicle = CHRONO_DATA_DIR +
              std::string("vehicle/hmmwv/vehicle/HMMWV_Vehicle.json");
    powertrain =
        CHRONO_DATA_DIR +
        std::string("vehicle/hmmwv/powertrain/HMMWV_ShaftsPowertrain.json");
    tire = CHRONO_DATA_DIR +
           std::string("vehicle/hmmwv/tire/HMMWV_TMeasyTire.json");
    zombie = CHRONO_DATA_DIR + std::string("synchrono/vehicle/HMMWV.json");
    lidar_pos = {2.3, 0, .4};
    cam_distance = 6.0;
    break;
  case VehicleType::SEDAN:
    vehicle = CHRONO_DATA_DIR +
              std::string("vehicle/sedan/vehicle/Sedan_Vehicle.json");
    powertrain =
        CHRONO_DATA_DIR +
        std::string("vehicle/sedan/powertrain/Sedan_SimpleMapPowertrain.json");
    tire = CHRONO_DATA_DIR +
           std::string("vehicle/sedan/tire/Sedan_TMeasyTire.json");
    zombie = CHRONO_DATA_DIR + std::string("synchrono/vehicle/Sedan.json");
    lidar_pos = {2.3, 0, .4};
    cam_distance = 6.0;
    break;
  }
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
