#ifndef CH_IDM_FOLLOWER_H
#define CH_IDM_FOLLOWER_H

#include "../../ChApiHil.h"
#include "../veh/Ch_8DOF_vehicle.h"
#include "ChROM_PathFollowerDriver.h"
#include <string>
#define MS_TO_MPH 2.23694
#define MPH_TO_MS 0.44704
#define AUDI_LENGTH 4.86
#define SUV_LENGTH 5.56
#define M_TO_MILE 0.000621371
#define MILE_TO_M 1609.34449789
namespace chrono {
namespace hil {

class CH_HIL_API ChROM_IDMFollower {
public:
  ChROM_IDMFollower(
      std::shared_ptr<Ch_8DOF_vehicle> rom, ///< associated vehicle
      std::shared_ptr<ChROM_PathFollowerDriver> path_follower,
      std::vector<double> params) ///< JSON file with piecewise params
  {
    m_rom = rom;
    m_path_follower = path_follower;
    m_params = params;
  }

  void Synchronize(double time, double step, double lead_distance,
                   double lead_speed);

private:
  std::shared_ptr<ChROM_PathFollowerDriver> m_path_follower;
  std::shared_ptr<Ch_8DOF_vehicle> m_rom;
  std::vector<double> m_params;

  ChVector<> previousPos;

  // traveldistance
  double dist;
  // theoretical speed
  double thero_speed = 0;
};

} // namespace hil
} // namespace chrono

#endif
