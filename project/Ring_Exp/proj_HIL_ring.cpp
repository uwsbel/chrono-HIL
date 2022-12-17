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

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include <map>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/controller/driver/SynMultiPathDriver.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/driver/ChIDM_Follower.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono/assets/ChLineShape.h"
#include "chrono/geometry/ChLineBezier.h"

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::sensor;
using namespace chrono::hil;
// =============================================================================
// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;
// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 25.0, 0.5);
ChQuaternion<> initRot = Q_from_AngZ(0);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY, PAC02)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
double terrainHeight = 0;   // terrain height (FLAT terrain only)
double terrainLength = 0.0; // size in X direction
double terrainWidth = 0.0;  // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double sim_time = 900.0;
double heartbeat = 0.1;
double step_size = 1e-3;
double tire_step_size = 1e-4;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50; // FPS = 50

// Debug logging
double debug_step_size = 1.0 / 1; // FPS = 1

// POV-Ray output
bool povray_output = false;

std::string path_file(std::string(STRINGIFY(HIL_DATA_DIR)) + "/ring/ring.txt");

const std::string out_dir = GetChronoOutputPath() + "ring_out";

// =============================================================================
void AddCommandLineOptions(ChCLI &cli) {
  // DDS Specific
  cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
  cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
  cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat",
                        std::to_string(heartbeat));
  cli.AddOption<std::vector<std::string>>(
      "DDS", "ip", "IP Addresses for initialPeersList", "127.0.0.1");
}

// =============================================================================

int main(int argc, char *argv[]) {
  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: "
           << CHRONO_VERSION << "\n\n";

  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));
  synchrono::SetDataPath(CHRONO_DATA_DIR + std::string("synchrono/"));

  std::string vehicle_filename =
      vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
  std::string powertrain_filename =
      vehicle::GetDataFile("sedan/powertrain/Sedan_SimpleMapPowertrain.json");
  std::string tire_filename =
      vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
  std::string zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");
  std::string steer_controller =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/ring/SteeringController.json";
  std::string speed_controller =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/ring/SpeedController.json";

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

  // Decide Vehicle Locations
  float deg_sec = (CH_C_PI * 1.5) / num_nodes;

  initLoc = ChVector<>(25.0 * cos(deg_sec * node_id),
                       25.0 * sin(deg_sec * node_id), 0.5);
  float rot_deg = deg_sec * node_id + CH_C_PI_2;
  if (rot_deg > CH_C_2PI) {
    rot_deg = rot_deg - CH_C_2PI;
  }
  initRot = Q_from_AngZ(rot_deg);

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
      tire->SetStepsize(step_size / 20);
      my_vehicle.InitializeTire(tire, wheel, tire_vis_type);
    }
  }

  // Create the terrain
  RigidTerrain terrain(my_vehicle.GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;
  patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);

  terrain.Initialize();

  // add terrain with weighted textures
  auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  terrain_mesh->LoadWavefrontMesh(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                      "/ring/ring_terrain.obj",
                                  false, true);
  terrain_mesh->Transform(ChVector<>(0, 0, 0),
                          ChMatrix33<>(1)); // scale to a different size
  auto terrain_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  terrain_shape->SetMesh(terrain_mesh);
  terrain_shape->SetName("terrain");
  terrain_shape->SetMutable(false);

  auto terrain_body = chrono_types::make_shared<ChBody>();
  terrain_body->SetPos({0, 0, 0.0});
  terrain_body->AddVisualShape(terrain_shape);
  terrain_body->SetBodyFixed(true);
  terrain_body->SetCollide(false);

  my_vehicle.GetSystem()->AddBody(terrain_body);

  // add dummy body
  auto attached_body = std::make_shared<ChBody>();
  my_vehicle.GetSystem()->AddBody(attached_body);
  attached_body->SetPos({0.0, 0.0, 0.0});
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // Add vehicle as an agent and initialize SynChronoManager
  syn_manager.AddAgent(chrono_types::make_shared<SynWheeledVehicleAgent>(
      &my_vehicle, zombie_filename));
  syn_manager.Initialize(my_vehicle.GetSystem());

  // Create a sensor manager
  // ====================================================
  auto manager =
      chrono_types::make_shared<ChSensorManager>(my_vehicle.GetSystem());
  float intensity = 1.2;
  manager->scene->AddPointLight({0, 0, 1e8}, {1.0, 1.0, 1.0}, 1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  if (node_id == 0) {
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        attached_body, // body camera is attached to
        30,            // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(0.0, 0.0, 50.0),
            Q_from_Euler123(ChVector<>(0.0, CH_C_PI_2, 0.0))), // offset pose
        1920,                                                  // image width
        1080,                                                  // image height
        1.608f,
        2); // fov, lag, exposure
    cam->SetName("Camera Sensor");

    // cam->PushFilter(
    //     chrono_types::make_shared<ChFilterVisualize>(1280, 720, "fov",
    //     false));
    //  Provide the host access to the RGBA8 buffer
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>("cam1/"));
    manager->AddSensor(cam);

    auto cam2 = chrono_types::make_shared<ChCameraSensor>(
        attached_body, // body camera is attached to
        30,            // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(0.0, 0.0, 15.0),
            Q_from_Euler123(ChVector<>(0.0, 0.5, 0.0))), // offset pose
        1920,                                            // image width
        1080,                                            // image height
        1.608f,
        2); // fov, lag, exposure
    cam2->SetName("Camera Sensor 2");

    // cam2->PushFilter(
    //    chrono_types::make_shared<ChFilterVisualize>(1920, 1080, "fov",
    //    false));
    //  Provide the host access to the RGBA8 buffer
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam2->PushFilter(chrono_types::make_shared<ChFilterSave>("cam2/"));
    manager->AddSensor(cam2);

    auto cam3 = chrono_types::make_shared<ChCameraSensor>(
        my_vehicle.GetChassis()->GetBody(), // body camera is attached to
        30,                                 // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(-6.0, 0.0, 3.0),
            Q_from_Euler123(ChVector<>(0.0, 0.13, 0.0))), // offset pose
        1920,                                             // image width
        1080,                                             // image height
        1.608f,
        2); // fov, lag, exposure
    cam3->SetName("Camera Sensor 3");

    // cam2->PushFilter(
    //    chrono_types::make_shared<ChFilterVisualize>(1920, 1080, "fov",
    //    false));
    //  Provide the host access to the RGBA8 buffer
    cam3->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam3->PushFilter(chrono_types::make_shared<ChFilterSave>("cam3/"));
    manager->AddSensor(cam3);
  }

  // -----------------
  // Initialize output
  // -----------------

  // Initialize output

  if (!filesystem::create_directory(filesystem::path(out_dir))) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }

  utils::CSV_writer csv(" ");

  // ------------------------
  // Create the driver system
  // ------------------------

  // read from a bezier curve, and form a closed loop
  auto path = ChBezierCurve::read(path_file, true);

  // idm parameters
  std::vector<double> followerParam;
  followerParam.push_back(8.9408);
  followerParam.push_back(1.5);
  followerParam.push_back(2.0);
  followerParam.push_back(2.0);
  followerParam.push_back(2.0);
  followerParam.push_back(4.0);
  followerParam.push_back(4.8895);

  ChIDMFollower driver(my_vehicle, steer_controller, speed_controller, path,
                       "road", 20.0 * MPH_TO_MS, followerParam);
  driver.Initialize();

  // ---------------
  // Simulation loop
  // ---------------

  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(render_step_size / step_size);

  // Initialize simulation frame counters
  int step_number = 0;
  int render_frame = 0;

  utils::ChRunningAverage RTF_filter(50);

  float *all_x = new float[num_nodes];
  float *all_y = new float[num_nodes];
  float *all_prev_x = new float[num_nodes];
  float *all_prev_y = new float[num_nodes];
  float *all_speed = new float[num_nodes];

  // csv labels
  csv << "time,";
  for (int j = 0; j < num_nodes; j++) {
    csv << "x_" + std::to_string(j) + ",";
    csv << "y_" + std::to_string(j) + ",";
    csv << "speed_" + std::to_string(j);
    if (j != num_nodes - 1) {
      csv << ",";
    }
  }
  csv << std::endl;

  double time = 0.0;

  // obtain and initiate all zombie instances
  std::map<AgentKey, std::shared_ptr<SynAgent>> zombie_map =
      syn_manager.GetZombies();
  std::map<int, std::shared_ptr<SynWheeledVehicleAgent>> id_map;
  for (std::map<AgentKey, std::shared_ptr<SynAgent>>::iterator it =
           zombie_map.begin();
       it != zombie_map.end(); ++it) {
    std::shared_ptr<SynAgent> temp_ptr = it->second;
    std::shared_ptr<SynWheeledVehicleAgent> converted_ptr =
        std::dynamic_pointer_cast<SynWheeledVehicleAgent>(temp_ptr);
    id_map.insert(std::make_pair(it->first.GetNodeID(), converted_ptr));
  }

  int lead_idx = (node_id + 1) % num_nodes;
  float act_dis = 0;

  while (time <= sim_time) {
    time = my_vehicle.GetSystem()->GetChTime();

    // update necessary zombie info for IDM
    if (step_number % int(heartbeat / step_size) == 0) {
      for (int i = 0; i < num_nodes; i++) {
        if (i != node_id) {
          ChVector<> temp_pos = id_map[i]->GetZombiePos();
          all_x[i] = temp_pos.x();
          all_y[i] = temp_pos.y();

          if (step_number == 0) {
            all_prev_x[i] = all_x[i];
            all_prev_y[i] = all_y[i];
          }

          all_speed[i] =
              sqrt((all_x[i] - all_prev_x[i]) * (all_x[i] - all_prev_x[i]) +
                   (all_y[i] - all_prev_y[i]) * (all_y[i] - all_prev_y[i])) /
              heartbeat;
          all_prev_x[i] = all_x[i];
          all_prev_y[i] = all_y[i];
        }
      }

      ChVector<> veh_pos = my_vehicle.GetPos();
      all_x[node_id] = veh_pos.x();
      all_y[node_id] = veh_pos.y();
      all_speed[node_id] = my_vehicle.GetSpeed();

      // Compute critical information
      float raw_dis = sqrt((all_x[node_id] - all_x[lead_idx]) *
                               (all_x[node_id] - all_x[lead_idx]) +
                           (all_y[node_id] - all_y[lead_idx]) *
                               (all_y[node_id] - all_y[lead_idx]));
      float temp = 1 - (raw_dis * raw_dis) / (2.0 * 25.0 * 25.0);
      if (temp > 1) {
        temp = 1;
      } else if (temp < -1) {
        temp = -1;
      }

      float theta = abs(acos(temp));
      act_dis = theta * 25.0;
    }

    if (step_number % 20 == 0) {
      csv << std::to_string(time) + ",";
    }

    // End simulation
    if (time >= t_end)
      break;

    // Render scene and output POV-Ray data
    if (node_id == 0 && (step_number % render_steps == 0)) {
      manager->Update();

      render_frame++;
    }

    if (node_id == 0 && step_number % 20 == 0) {
      for (int j = 0; j < num_nodes; j++) {
        csv << std::to_string(all_x[j]) + ",";
        csv << std::to_string(all_y[j]) + ",";
        csv << std::to_string(all_speed[j]);
        if (j != num_nodes - 1) {
          csv << ",";
        }
      }
      csv << std::endl;
      csv.write_to_file(out_dir + "/ring_save.csv");
    }

    // Get driver inputs
    DriverInputs driver_inputs = driver.GetInputs();

    // Update modules (process inputs from other modules)
    syn_manager.Synchronize(time);
    driver.Synchronize(time, step_size, act_dis, all_speed[lead_idx]);
    terrain.Synchronize(time);
    my_vehicle.Synchronize(time, driver_inputs, terrain);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);
    terrain.Advance(step_size);
    my_vehicle.Advance(step_size);

    // std::cout << my_sedan.GetVehicle().GetPos() << std::endl;

    // Increment frame number
    step_number++;

    if (syn_manager.IsOk()) {
      syn_manager.QuitSimulation();
    }
  }

  return 0;
}