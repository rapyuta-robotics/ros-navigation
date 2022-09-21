#pragma once

#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner {

class PreferFastSpinCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};
};

} /* namespace base_local_planner */
