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

#include "chrono_hil/ROM/syn/Ch_8DOF_zombie.h"

#include "chrono_hil/network/tcp/ChTCPClient.h"
#include "chrono_hil/network/tcp/ChTCPServer.h"

// =============================================================================

// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::hil;
// =============================================================================
const double RADS_2_RPM = 30 / CH_C_PI;
const double MS_2_MPH = 2.2369;
// =====================================================

struct rom_item {
  int type;
  ChVector<double> pos;
  ChVector<double> rot;
  int path;
  int ld_id;
  int idm_type;
};

// =====================================================

void ReadRomInitFile(std::string csv_filename, std::vector<rom_item> &rom_arr);
void AddCommandLineOptions(ChCLI &cli);

// =====================================================

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
double heartbeat = 4e-2;

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

std::string demo_data_path = std::string(STRINGIFY(HIL_DATA_DIR));

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

  syn_manager.SetHeartbeat(heartbeat);

  // ===========================================

  // all the demo data will be in user-specified location
  SetChronoDataPath(demo_data_path);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));
  synchrono::SetDataPath(CHRONO_DATA_DIR + std::string("synchrono/"));

  // read rom data csv
  std::vector<rom_item> rom_data;
  std::string filename = std::string(STRINGIFY(HIL_DATA_DIR)) +
                         "/Environments/SanFrancisco/rom_init/test.csv";
  std::cout << filename << std::endl;
  ReadRomInitFile(filename, rom_data);

  // --------------
  // Create vehicle
  // --------------

  std::string vehicle_filename =
      vehicle::GetDataFile("audi/json/audi_Vehicle.json");
  std::string powertrain_filename =
      vehicle::GetDataFile("audi/json/audi_SimpleMapPowertrain.json");
  std::string tire_filename =
      vehicle::GetDataFile("audi/json/audi_Pac02Tire.json");
  std::string zombie_filename =
      CHRONO_DATA_DIR + std::string("synchrono/vehicle/Sedan.json");

  // Initial vehicle location and orientation
  ChVector<> initLoc;
  ChQuaternion<> initRot;
  if (node_id == 0) {
    initLoc = ChVector<>(930.434, -150.87, -65.2);
    ChQuaternion<> initRot = Q_from_AngZ(3.14 / 2);
  } else if (node_id == 1) {
    initLoc = ChVector<>(930.434, -50.87, -65.2);
    ChQuaternion<> initRot = Q_from_AngZ(3.14 / 2);
  } else if (node_id == 2) {
    initLoc = ChVector<>(930.434, 0, -65.2);
    ChQuaternion<> initRot = Q_from_AngZ(3.14 / 2);
  }

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

  // Add vehicle as an agent and initialize SynChronoManager
  auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(
      &my_vehicle, zombie_filename);
  syn_manager.AddAgent(agent);
  syn_manager.Initialize(my_vehicle.GetSystem());

  auto attached_body = std::make_shared<ChBody>();
  my_vehicle.GetSystem()->AddBody(attached_body);
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);
  auto box = chrono_types::make_shared<ChBoxShape>();
  box->GetBoxGeometry().Size = ChVector<>(10000.0, 10000.0, 0.001);
  auto black_mat = chrono_types::make_shared<ChVisualMaterial>();
  black_mat->SetDiffuseColor(ChColor(0.3f, 0.3f, 0.3f));
  box->AddMaterial(black_mat);
  attached_body->AddVisualShape(box, ChFrame<>(ChVector<>(0, 0, -65.6), QUNIT));

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
  std::shared_ptr<ChSensorManager> manager;

  if (node_id == 1) {
    manager =
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
                                Q_from_AngAxis(0.0, {0, 1, 0})), // offset pose
        1280,                                                    // image width
        720,                                                     // image height
        3.14 / 1.5,                                              // fov
        1);

    driver_cam->SetName("DriverCam");
    driver_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
        1280, 720, "Camera1", false));
    driver_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(driver_cam);
  }

  // --------------
  // Create control
  // --------------
  /*
  ChSDLInterface SDLDriver;

  SDLDriver.Initialize();

  SDLDriver.SetJoystickConfigFile(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                  std::string("/joystick/controller_G27.json"));
  */
  std::string path_file("paths/output.txt");

  auto path = ChBezierCurve::read(demo_data_path + "/paths/output.txt", true);

  ChPathFollowerDriver driver(my_vehicle, path, "my_path", 6.0);
  driver.GetSteeringController().SetLookAheadDistance(5);
  driver.GetSteeringController().SetGains(0.2, 0, 0);
  driver.GetSpeedController().SetGains(0.4, 0, 0);
  driver.Initialize();

  // Create Zombies
  int num_rom = rom_data.size();

  std::string hmmwv_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";
  std::string sedan_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/sedan/sedan_rom.json";
  std::string patrol_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/patrol/patrol_rom.json";
  std::string audi_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/audi/audi_rom.json";

  std::vector<std::shared_ptr<Ch_8DOF_zombie>> zombie_vec;
  for (int i = 0; i < num_rom; i++) {
    std::string rom_json;
    if (rom_data[i].type == 0) {
      rom_json = hmmwv_json;
    } else if (rom_data[i].type == 1) {
      rom_json = sedan_json;
    } else if (rom_data[i].type == 2) {
      rom_json = patrol_json;
    } else if (rom_data[i].type == 3) {
      rom_json = audi_json;
    }

    auto rom_zombie = chrono_types::make_shared<Ch_8DOF_zombie>(rom_json, 0.0);
    zombie_vec.push_back(rom_zombie);
    rom_zombie->Initialize(my_vehicle.GetSystem());
    std::cout << "zombie: " << i << std::endl;
  }

  // Create TCP Tunnel
  ChTCPClient chrono_0("127.0.0.1", 1204,
                       num_rom * 11); // create a TCP client for rank 0
  ChTCPClient chrono_1("127.0.0.1", 1205,
                       num_rom * 11); // create a TCP client for rank 1
  ChTCPClient chrono_2("127.0.0.1", 1206,
                       num_rom * 11); // create a TCP client for rank 2

  if (node_id == 0) {
    chrono_0.Initialize(); // initialize TCP connection for rank 1
  } else if (node_id == 1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        5000)); // wait for 2000 seconds to prevent duplicated connection
    chrono_1.Initialize(); // initialize TCP connection for rank 2
  } else if (node_id == 2) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        10000)); // wait for 2000 seconds to prevent duplicated connection
    chrono_2.Initialize(); // initialize TCP connection for rank 2
  }

  double sim_time = 0.f;
  double last_time = 0.f;
  int step_number = 0;
  if (node_id == 1) {
    manager->Update();
  }

  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();

  while (syn_manager.IsOk()) {
    sim_time = my_vehicle.GetSystem()->GetChTime();

    // Get driver inputs
    DriverInputs driver_inputs;
    // driver_inputs.m_throttle = SDLDriver.GetThrottle();
    // driver_inputs.m_steering = SDLDriver.GetSteering();
    // driver_inputs.m_braking = SDLDriver.GetBraking();

    driver_inputs = driver.GetInputs();
    /*
    if (SDLDriver.Synchronize() == 1)
      break;
*/
    // Update modules (process inputs from other modules)
    driver.Synchronize(sim_time);
    my_vehicle.Synchronize(sim_time, driver_inputs, terrain);
    terrain.Synchronize(sim_time);
    syn_manager.Synchronize(sim_time);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);
    my_vehicle.Advance(step_size);
    terrain.Advance(step_size);

    if (node_id == 1) {
      manager->Update();
    }

    // Synchronize
    if (step_number == 0) {
      std::vector<float> data_to_send;
      data_to_send.push_back(1.0);
      data_to_send.push_back(1.0);
      data_to_send.push_back(1.0);
      if (node_id == 0) {
        chrono_0.Write(data_to_send);
      } else if (node_id == 1) {
        chrono_1.Write(data_to_send);
      } else if (node_id == 2) {
        chrono_2.Write(data_to_send);
      }
    }

    // Update zombies
    // ==================================================
    // TCP Synchronization Section
    // ==================================================
    // read data from rom distributor 1
    if (step_number % 20 == 0) {
      if (node_id == 0) {
        chrono_0.Read();
      } else if (node_id == 1) {
        chrono_1.Read();
      } else if (node_id == 2) {
        chrono_2.Read();
      }

      std::vector<float> recv_data;
      if (node_id == 0) {
        recv_data = chrono_0.GetRecvData();
      } else if (node_id == 1) {
        recv_data = chrono_1.GetRecvData();
      } else if (node_id == 2) {
        recv_data = chrono_2.GetRecvData();
      }

      for (int i = 0; i < num_rom; i++) {
        zombie_vec[i]->Update(
            ChVector<>(recv_data[0 + i * 11], recv_data[1 + i * 11],
                       recv_data[2 + i * 11]),
            ChVector<>(recv_data[3 + i * 11], recv_data[4 + i * 11],
                       recv_data[5 + i * 11]),
            recv_data[6 + i * 11], recv_data[7 + i * 11], recv_data[8 + i * 11],
            recv_data[9 + i * 11], recv_data[10 + i * 11]);
      }

      // send data to distributor 1
      std::vector<float> data_to_send;
      data_to_send.push_back(my_vehicle.GetChassis()->GetPos().x());
      data_to_send.push_back(my_vehicle.GetChassis()->GetPos().y());
      data_to_send.push_back(my_vehicle.GetChassis()->GetPos().z());

      if (node_id == 0) {
        chrono_0.Write(data_to_send);
      } else if (node_id == 1) {
        chrono_1.Write(data_to_send);
      } else if (node_id == 2) {
        chrono_2.Write(data_to_send);
      }
      std::cout << my_vehicle.GetChassis()->GetPos() << std::endl;
    }

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
  // Properly shuts down other ranks when one rank ends early
  syn_manager.QuitSimulation();
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

void ReadRomInitFile(std::string csv_filename, std::vector<rom_item> &rom_vec) {
  std::ifstream inputFile(csv_filename);

  if (!inputFile.is_open()) {
    std::cout << "Failed to open the file." << std::endl;
  }

  std::vector<std::vector<float>> data; // 2D vector to store the float numbers

  std::string line;

  int line_count = 0;

  while (std::getline(inputFile, line)) {
    if (line_count != 0) {
      std::istringstream iss(line);
      std::string token;
      std::vector<float> row; // Vector to store a row of float numbers
      while (std::getline(iss, token, ',')) { // Assuming comma as the delimiter
        float number;
        try {
          number = std::stof(token); // Convert string to float
          row.push_back(number);     // Store the float number in the row vector
        } catch (const std::exception &e) {
          // Failed to convert to float, ignore and continue
        }
      }
      data.push_back(row); // Add the row vector to the 2D vector
    }

    line_count++;
  }

  inputFile.close(); // Close the input file

  for (int i = 0; i < data.size(); i++) {
    rom_item temp_struct;
    temp_struct.type = int(data[i][1]);
    temp_struct.pos =
        ChVector<>(float(data[i][2]), float(data[i][3]), float(data[i][4]));

    temp_struct.rot =
        ChVector<>(float(data[i][5]), float(data[i][6]), float(data[i][7]));

    temp_struct.path = int(data[i][8]);
    temp_struct.ld_id = int(data[i][9]);
    temp_struct.idm_type = int(data[i][10]);
    rom_vec.push_back(temp_struct);
  }

  // Print the float numbers in the 2D vector
  std::cout << "test struct print" << std::endl;
  for (auto temp_data : rom_vec) {
    std::cout << temp_data.type << ", " << temp_data.pos << ", "
              << temp_data.rot << ", " << temp_data.path << std::endl;
  }
}

void AddCommandLineOptions(ChCLI &cli) {
  // DDS Specific
  cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
  cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
  cli.AddOption<std::vector<std::string>>(
      "DDS", "ip", "IP Addresses for initialPeersList", "127.0.0.1");
}
