
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_hil/ROM/SimplifiedVehicle_2DOF.h"
#include "chrono_hil/driver/ChSDLInterface.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include <irrlicht.h>

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::hil;
using namespace chrono::vehicle;

int main(int argc, char *argv[]) {
  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: "
           << CHRONO_VERSION << "\n\n";

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // Create a ChronoENGINE physical system
  ChSystemNSC sys;

  // Create the a plane using body of 'box' type:
  auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
  ground_mat->SetFriction(0.5f);

  auto mrigidBodyB = chrono_types::make_shared<ChBodyEasyBox>(
      50, 1, 50, 1000, true, true, ground_mat);
  mrigidBodyB->SetBodyFixed(true);
  mrigidBodyB->SetPos(ChVector<>(0, -0.5, 0));
  sys.Add(mrigidBodyB);

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
  vehicle_shell->SetPos(ChVector<>(0, 0.25, 0));

  ChQuaternion<> initRot = ChQuaternion<>(0, 0, 0, 0);
  initRot.Q_from_AngX(-CH_C_PI / 2);
  vehicle_shell->SetRot(initRot);

  vehicle_shell->SetBodyFixed(true);
  vehicle_shell->AddVisualShape(suv_trimesh_shape);
  sys.Add(vehicle_shell);

  SimplifiedVehicle_2DOF test_vehicle = SimplifiedVehicle_2DOF();

  // Create the Irrlicht visualization sys
  auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
  vis->AttachSystem(&sys);
  vis->SetWindowSize(800, 600);
  vis->SetWindowTitle("Convex decomposed wheel");
  vis->Initialize();
  vis->AddCamera(ChVector<>(3.5, 2.5, -2.4));
  vis->AddTypicalLights();

  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  SDLDriver.SetJoystickConfigFile(
      vehicle::GetDataFile("joystick/controller_G27.json"));

  // Simulation loop
  double timestep = 0.002;
  double sim_time = 0.0;
  ChRealtimeStepTimer realtime_timer;
  while (vis->Run()) {

    // Driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_steering = SDLDriver.GetSteering();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    std::cout << "throttle: " << driver_inputs.m_throttle
              << ", brake: " << driver_inputs.m_braking
              << ", steer:" << driver_inputs.m_steering << std::endl;

    test_vehicle.Step(driver_inputs.m_throttle, driver_inputs.m_braking,
                      driver_inputs.m_steering, timestep);

    float x_pos = test_vehicle.GetXPos();
    float y_pos = test_vehicle.GetYPos();
    float yaw = test_vehicle.GetYaw();

    initRot.Q_from_Euler123(ChVector<>(-CH_C_PI / 2, -yaw, 0));

    vehicle_shell->SetPos(ChVector<>(x_pos, 0.25, y_pos));
    vehicle_shell->SetRot(initRot);

    vis->BeginScene();
    vis->Render();
    vis->EndScene();
    sys.DoStepDynamics(timestep);
    realtime_timer.Spin(timestep);
    sim_time = sim_time + timestep;
  }

  return 0;
}
