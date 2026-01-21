#include "base_local_planner/magnitude_cost_function.h"
#include <cmath>

namespace base_local_planner {

MagnitudeCostFunction::MagnitudeCostFunction()
{

}

double MagnitudeCostFunction::scoreTrajectory(Trajectory &traj) {
	double used_v = std::max(0.01, fabs(traj.xv_));
	return ((1.0 / used_v) * getScale());
}

bool MagnitudeCostFunction::prepare()
{
	return true;
}

} /* namespace base_local_planner */
