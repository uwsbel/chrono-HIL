
#ifndef CH_EIGHT_ROM_H
#define CH_EIGHT_ROM_H

#include "../ChApiHil.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "rom_Eightdof.h"
#include <string>

using namespace chrono;
using namespace chrono::vehicle;

class Ch_8DOF_vehicle {

public:
  Ch_8DOF_vehicle(std::string vehicle_json, std::string tire_json,
                  float z_plane);

  void Advance(float time, DriverInputs inputs);

  ChVector<> GetPos();
  ChQuaternion<> GetRot();
  float GetStepSize();

private:
  float rom_z_plane;

  VehicleState veh1_st;
  VehicleParam veh1_param;

  // lets define our tires, we have 4 different
  // tires so 4 states
  TMeasyState tirelf_st;
  TMeasyState tirerf_st;
  TMeasyState tirelr_st;
  TMeasyState tirerr_st;

  // but all of them have the same parameters
  // so only one parameter structure
  TMeasyParam tire_param;
};

#endif
