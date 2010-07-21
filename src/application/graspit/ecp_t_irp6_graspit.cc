#include <stdio.h>
#include <string>
#include <unistd.h>
#include <iostream>

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "generator/ecp/ecp_g_constant_velocity.h"
#include "ecp_t_irp6_graspit.h"
#include "ecp_mp_t_graspit.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
irp6_grasp::irp6_grasp(lib::configurator &_config) :
	task(_config)
{

	if (config.section_name == ECP_IRP6OT_M_SECTION) {
		ecp_m_robot = new irp6ot_m::robot(*this);
	} else if (config.section_name == ECP_IRP6P_M_SECTION) {
		ecp_m_robot = new irp6p_m::robot(*this);
	}

	cvgenjoint = new generator::constant_velocity(*this, lib::ECP_JOINT, 6);
	cvgenjoint->set_debug(true);

	sr_ecp_msg->message("ecp IRP6 loaded");
}
;

void irp6_grasp::main_task_algorithm(void)
{

	sr_ecp_msg->message("ecp IRP6 ready");

	struct _irp6{
		double joint[6];
	} mp_ecp_irp6_command;
	vector<double> coordinates1(6);

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	for (;;) {
			sr_ecp_msg->message("Waiting for MP order");

			get_next_state();

			sr_ecp_msg->message("Order received");
			flushall();

			if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_IRP6)  {

				sr_ecp_msg->message("ECP_GEN_IRP6");

				memcpy(&mp_ecp_irp6_command, mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_irp6_command));
				for (int i=0; i<6; ++i)
					coordinates1[i] = mp_ecp_irp6_command.joint[i];

				cvgenjoint->reset();
				cvgenjoint->set_absolute();
				cvgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
				if (cvgenjoint->calculate_interpolate())
					cvgenjoint->Move();
			}

			ecp_termination_notice();
	}

	ecp_termination_notice();
}
;

} // namespace task
} // namespace common

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::irp6_grasp(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

