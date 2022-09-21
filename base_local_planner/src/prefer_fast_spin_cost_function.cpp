#include <base_local_planner/prefer_fast_spin_cost_function.h>
#include <cmath>

namespace base_local_planner {

double PreferFastSpinCostFunction::scoreTrajectory(Trajectory &traj) {
  const double mag = std::hypot(traj.xv_, traj.yv_);
  const double abs_thetav = abs(traj.thetav_);
  const double turn_radius =  mag / abs_thetav;
  const bool spin_in_place = turn_radius < 0.05;
  
  return spin_in_place ? 1 / abs_thetav : 0;
}

} /* namespace base_local_planner */
