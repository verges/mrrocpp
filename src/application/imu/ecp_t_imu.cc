/*!
 * @file
 * @brief File contains edge_follow_mr ecp_task class definition of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include <cstdio>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "robot/irp6p_m/const_irp6p_m.h"

#include "ecp_t_imu.h"

#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"
#include "generator/ecp/tff_nose_run/ecp_g_tff_nose_run.h"

#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
ecp_imu::ecp_imu(lib::configurator &_config) :
		common::task::task(_config)
{
	// the robot is choose depending on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		// TODO: throw
	}

	// utworzenie generatorow do uruchamiania dispatcherem

	register_generator(new common::generator::bias_edp_force(*this));

	{
		common::generator::tff_nose_run *ecp_gen = new common::generator::tff_nose_run(*this, 8);
		ecp_gen->configure_pulse_check(false);
		ecp_gen->configure_behaviour(lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::CONTACT, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION);
		ecp_gen->configure_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		register_generator(ecp_gen);
	}

	// utworzenie podzadan

	sr_ecp_msg->message("ecp imu loaded");
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::ecp_imu(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
