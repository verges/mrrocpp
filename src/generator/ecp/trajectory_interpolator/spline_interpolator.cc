/**
 * @file
 * @brief Contains definitions of the methods of spline_interpolator class.
 * @author rtulwin
 * @ingroup generators
 */

#include "spline_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

using namespace std;

spline_interpolator::spline_interpolator()
{
    // TODO Auto-generated constructor stub
}

spline_interpolator::~spline_interpolator()
{
    // TODO Auto-generated destructor stub
}

bool spline_interpolator::interpolate_relative_pose(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {

    return interpolate_absolute_pose(it, cv, mc);
}

bool spline_interpolator::interpolate_absolute_pose(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {

    vector<double> coordinates (it->axes_num);
    for (int i = 0; i < it->interpolation_node_no; i++) {
            for (int j = 0; j < it->axes_num; j++) {
                if (it->type == 1)//linear
                {
                    coordinates[j] = calculate_position(it, j, (i+1) * mc);
                }
                else if (it->type == 2)//cubic
                {
                    coordinates[j] = calculate_velocity(it, j, (i+1) * mc) * mc;
                }
                else if (it->type == 3)//quintic
                {
                    coordinates[j] = 0.5 * calculate_acceleration(it, j, (i+1) * mc) * mc * mc;
                }
            }
            cv.push_back(coordinates);
    }

    return true;
}

double spline_interpolator::generate_relative_coordinate(int node_counter, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, int axis_num, double mc)
{


    return 0.0;
}

double spline_interpolator::calculate_position(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time)
{
  double t[6];
  double position;
  generatePowers(5, time, t);

  position = t[0]*it->coeffs[i][0] +
             t[1]*it->coeffs[i][1] +
             t[2]*it->coeffs[i][2] +
             t[3]*it->coeffs[i][3] +
             t[4]*it->coeffs[i][4] +
             t[5]*it->coeffs[i][5];
  return position;
}

double spline_interpolator::calculate_velocity(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time)
{
  double t[5];
  double velocity;
  generatePowers(4, time, t);

  velocity = t[0]*it->coeffs[i][0] +
             2.0*t[1]*it->coeffs[i][1] +
             3.0*t[2]*it->coeffs[i][2] +
             4.0*t[3]*it->coeffs[i][3] +
             5.0*t[4]*it->coeffs[i][4];
  return velocity;
}

double spline_interpolator::calculate_acceleration(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time)
{
  double t[4];
  double acceleration;
  generatePowers(3, time, t);

  acceleration = 2.0*t[0]*it->coeffs[i][0] +
                 6.0*t[1]*it->coeffs[i][1] +
                 12.0*t[2]*it->coeffs[i][2] +
                 20.0*t[3]*it->coeffs[i][3];
  return acceleration;
}

inline void spline_interpolator::generatePowers(int power, double x, double * powers)
{
  powers[0] = 1.0;
  for (int i = 1; i <= power; i++)
  {
    powers[i] = powers[i-1] * x;
  }
  return;
}

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
