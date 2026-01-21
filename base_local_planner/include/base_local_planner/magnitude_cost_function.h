
#ifndef MAGNITUDE_COST_FUNCTION_H_
#define MAGNITUDE_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory.h>


namespace base_local_planner {


 /**
  * @brief Gives higher score to trajectories with a higher linear velocity
 */
class MagnitudeCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

  MagnitudeCostFunction();

  ~MagnitudeCostFunction() {}

  double scoreTrajectory(Trajectory &traj);

  bool prepare();

};

} /* namespace base_local_planner */
#endif /* MAGNITUDE_COST_FUNCTION_H_ */
