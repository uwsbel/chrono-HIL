#include "chrono_hil/ROM/rom_Eightdof.h"
#include "chrono_hil/ROM/rom_TMeasy.h"
#include "chrono_hil/ROM/rom_utils.h"
#include <chrono>
#include <iostream>
#include <stdint.h>

int main(int argc, char *argv[]) {

  // Input files
  std::string fileName =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/test_set2.txt";
  // std::string fileName = "./inputs/test_set1.txt";
  // std::string fileName = "./inputs/st.txt";
  // std::string fileName = "./inputs/acc.txt";

  // Vehicle parameters JSON file
  std::string vehParamsJSON =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/HMMWV.json";

  // Tire parameters JSON file
  std::string tireParamsJSON =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/TMeasy.json";

  std::vector<Entry> driverData;
  driverInput(driverData, fileName);

  // initialize controls to 0
  std::vector<double> controls(4, 0);

  // lets initialize a vehicle struct and see where we land

  VehicleState veh1_st;
  VehicleParam veh1_param;

  // Set vehicle parameters from JSON file
  setVehParamsJSON(veh1_param, vehParamsJSON);
  vehInit(veh1_st, veh1_param);

  // lets define our tires, we have 4 different
  // tires so 4 states
  TMeasyState tirelf_st;
  TMeasyState tirerf_st;
  TMeasyState tirelr_st;
  TMeasyState tirerr_st;

  // but all of them have the same parameters
  // so only one parameter structure
  TMeasyParam tire_param;

  // set the tire parameters from a JSON file
  setTireParamsJSON(tire_param, tireParamsJSON);

  // // now we initialize each of our parameters
  tireInit(tire_param);

  // double endTime = 14.509;
  // double endTime = 11.609;
  double endTime = 10.009;
  // double endTime = 8.509;

  double step = veh1_param._step;

  // now lets run our simulation
  double t = 0;
  int timeStepNo = 0; // time step counter

  while (t < endTime) {
    // get the controls for this time step
    getControls(controls, driverData, t);

    std::cout << "t:" << t << std::endl;
    for (int i = 0; i < controls.size(); i++) {
      std::cout << controls[i] << ",";
    }
    std::cout << std::endl;

    // transform velocities and other needed quantities from
    // vehicle frame to tire frame
    vehToTireTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh1_st,
                       veh1_param, controls);

    // advance our 4 tires
    tireAdv(tirelf_st, tire_param, veh1_st, veh1_param, controls);
    tireAdv(tirerf_st, tire_param, veh1_st, veh1_param, controls);

    // modify controls for our rear tires as they dont take steering
    std::vector<double> mod_controls = {controls[0], 0, controls[2],
                                        controls[3]};
    tireAdv(tirelr_st, tire_param, veh1_st, veh1_param, mod_controls);
    tireAdv(tirerr_st, tire_param, veh1_st, veh1_param, mod_controls);

    // transform tire forces to vehicle frame
    tireToVehTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh1_st,
                       veh1_param, controls);

    // copy the useful stuff that needs to be passed onto the vehicle
    std::vector<double> fx = {tirelf_st._fx, tirerf_st._fx, tirelr_st._fx,
                              tirerr_st._fx};
    std::vector<double> fy = {tirelf_st._fy, tirerf_st._fy, tirelr_st._fy,
                              tirerr_st._fy};
    double huf = tirelf_st._rStat;
    double hur = tirerr_st._rStat;

    vehAdv(veh1_st, veh1_param, fx, fy, huf, hur);

    std::cout << veh1_st._x << "," << veh1_st._y << std::endl;

    t += step;
    timeStepNo += 1;
  }
  return 0;
}