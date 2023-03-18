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
// This demo conducts a scaling test on the 8dof vehicle model
// =============================================================================

#include <chrono>
#include <iostream>
#include <stdint.h>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include <irrlicht.h>
#include <time.h>

#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_hil/ROM/driver/ChROM_IDMFollower.h"
#include "chrono_hil/ROM/driver/ChROM_PathFollowerDriver.h"
#include "chrono_hil/ROM/syn/Ch_8DOF_zombie.h"
#include "chrono_hil/ROM/veh/Ch_8DOF_vehicle.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono/core/ChBezierCurve.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_hil/network/ChBoostInStreamer.h"
#include "chrono_hil/network/ChBoostOutStreamer.h"

#include "chrono_hil/timer/ChRealtimeCumulative.h"
// =============================================================================
// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

#define IP_OUT "127.0.0.1"
#define PORT_OUT 1204
#define PORT_IN 1204
#define PORT_SIG_OUT 1209
#define PORT_SIG_IN 1209
// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;
using namespace chrono::hil;
using namespace chrono::synchrono;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50; // FPS = 50

ChVector<> initLoc(0, 0, 1.4);
ChQuaternion<> initRot(1, 0, 0, 0);

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// =============================================================================
void AddCommandLineOptions(ChCLI &cli) {
  // DDS Specific
  cli.AddOption<int>("syn", "d,node_id", "ID for this Node", "0");
  cli.AddOption<int>("syn", "n,num_nodes", "Number of Nodes", "1");
  cli.AddOption<std::vector<std::string>>(
      "DDS", "ip", "IP Addresses for initialPeersList", "127.0.0.1");
}

int main(int argc, char *argv[]) {
  // read from cli
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  if (!cli.Parse(argc, argv, true))
    return 0;

  const int node_id = cli.GetAsType<int>("node_id");
  const int num_nodes = cli.GetAsType<int>("num_nodes");
  const std::vector<std::string> ip_list =
      cli.GetAsType<std::vector<std::string>>("ip");

  ChSystemSMC my_system;
  my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
  int num_rom = 50;

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // Vehicle, node_id = 1
  std::string vehicle_filename =
      vehicle::GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json");
  std::string tire_filename =
      vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json");
  std::string powertrain_filename =
      vehicle::GetDataFile("hmmwv/powertrain/HMMWV_ShaftsPowertrain.json");

  std::vector<std::shared_ptr<Ch_8DOF_vehicle>>
      rom_vec; // rom vector, for node 0
  std::vector<std::shared_ptr<Ch_8DOF_zombie>> zombie_vec;

  // Create the Sedan vehicle, set parameters, and initialize
  WheeledVehicle my_vehicle((ChSystem *)&my_system, vehicle_filename);

  if (node_id == 0) {
    float init_height = 0.45;
    std::string rom_json =
        std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";

    for (int i = 0; i < num_rom; i++) {
      std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
          chrono_types::make_shared<Ch_8DOF_vehicle>(rom_json, init_height,
                                                     step_size);
      rom_veh->SetInitPos(initLoc +
                          ChVector<>(0.0, 0.0 + i * 2.0, init_height));
      rom_veh->SetInitRot(0.0);
      rom_veh->Initialize(&my_system);
      rom_vec.push_back(rom_veh);
      std::cout << "initialize: " << i << std::endl;
    }
  } else if (node_id == 1) {
    auto ego_chassis = my_vehicle.GetChassis();
    my_vehicle.Initialize(ChCoordsys<>(
        initLoc + ChVector<>(0.0, 0.0 + num_rom * 2.0, 0.25), initRot));
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
        tire->SetStepsize(step_size / 2);
        my_vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
      }
    }

    float init_height = 0.45;
    std::string rom_json =
        std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";

    for (int i = 0; i < num_rom; i++) {
      auto rom_zombie =
          chrono_types::make_shared<Ch_8DOF_zombie>(rom_json, init_height);
      zombie_vec.push_back(rom_zombie);
      rom_zombie->Initialize(&my_system);
      std::cout << "zombie: " << i << std::endl;
    }
  }

  // Initialize terrain
  RigidTerrain terrain(&my_system);

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
  my_system.AddBody(attached_body);
  attached_body->SetPos(ChVector<>(0.0, 0.0, 0.0));
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // Create camera
  // Create the camera sensor
  auto manager = chrono_types::make_shared<ChSensorManager>(&my_system);
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
          ChVector<>(20.0, -35.0, 1.0),
          Q_from_Euler123(ChVector<>(0.0, 0.0, C_PI / 2))), // offset pose
      1920,                                                 // image width
      1080,                                                 // image
      1.608f, 1); // fov, lag, exposure cam->SetName("Camera Sensor");

  cam->PushFilter(
      chrono_types::make_shared<ChFilterVisualize>(1920, 1080, "test", false));
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

  // Simulation end time
  double t_end = 14.0;

  // create boost data streaming interface
  ChRealtimeCumulative realtime_timer;

  while (time <= t_end) {

    time = my_system.GetChTime();

    if (step_number == 1) {
      realtime_timer.Reset();
    } else if (step_number != 1 && step_number != 0) {
      realtime_timer.Spin(time);
    }
    // std::cout << "time: " << time << std::endl;

    // End simulation
    if (time >= t_end)
      break;

    // Driver inputs
    DriverInputs driver_inputs;

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
      driver_inputs.m_braking = 0.6;
      driver_inputs.m_steering = 0.0;
    } else {
      driver_inputs.m_throttle = 0.0;
      driver_inputs.m_braking = 0.0;
      driver_inputs.m_steering = 0.0;
    }

    // Update modules (process inputs from other modules)
    // terrain.Synchronize(time);

    // Advance simulation for one timestep for all modules
    if (node_id == 0) {
      if (step_number == 0) {
        ChBoostInStreamer boost_receiver(PORT_IN, 1);
        std::cout << "waiting for start signal" << std::endl;
        boost_receiver.Synchronize();
        std::cout << "signal received" << std::endl;
        std::vector<float> recv_data;
        recv_data = boost_receiver.GetRecvData();
      }

      ChBoostOutStreamer boost_streamer(IP_OUT, PORT_OUT);
      for (int i = 0; i < num_rom; i++) {
        rom_vec[i]->Advance(time, driver_inputs);
        ChVector<> rom_pos = rom_vec[i]->GetPos();
        ChQuaternion<> rom_rot = rom_vec[i]->GetRot();
        ChVector<> rom_rot_vec = rom_rot.Q_to_Euler123();
        DriverInputs rom_inputs = rom_vec[i]->GetDriverInputs();

        boost_streamer.AddData(rom_pos.x());
        boost_streamer.AddData(rom_pos.y());
        boost_streamer.AddData(rom_pos.z());
        boost_streamer.AddData(rom_rot_vec.x());
        boost_streamer.AddData(rom_rot_vec.y());
        boost_streamer.AddData(rom_rot_vec.z());
        boost_streamer.AddData(rom_inputs.m_steering);
        boost_streamer.AddData(rom_vec[i]->GetTireRotation(0));
        boost_streamer.AddData(rom_vec[i]->GetTireRotation(1));
        boost_streamer.AddData(rom_vec[i]->GetTireRotation(2));
        boost_streamer.AddData(rom_vec[i]->GetTireRotation(3));
      }
      boost_streamer.Synchronize();
    } else if (node_id == 1) {
      if (step_number == 0) {
        ChBoostOutStreamer boost_streamer(IP_OUT, PORT_OUT);
        boost_streamer.AddData(1.0);
        boost_streamer.Synchronize();
      }

      my_vehicle.Synchronize(time, driver_inputs, terrain);
      my_vehicle.Advance(step_size);

      ChBoostInStreamer boost_receiver(PORT_IN, 550);
      boost_receiver.Synchronize();
      std::vector<float> recv_data;
      recv_data = boost_receiver.GetRecvData();
      for (int i = 0; i < num_rom; i++) {
        zombie_vec[i]->Update(
            ChVector<>(recv_data[0 + i * 11], recv_data[1 + i * 11],
                       recv_data[2 + i * 11]),
            ChVector<>(recv_data[3 + i * 11], recv_data[4 + i * 11],
                       recv_data[5 + i * 11]),
            recv_data[6 + i * 11], recv_data[7 + i * 11], recv_data[8 + i * 11],
            recv_data[9 + i * 11], recv_data[10 + i * 11]);
      }
    }

    terrain.Advance(step_size);
    my_system.DoStepDynamics(step_size);

    std::cout << "time:" << time << std::endl;

    // manager->Update();

    // Increment frame number
    step_number++;
  }
}