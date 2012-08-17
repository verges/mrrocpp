/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>

#include <unistd.h>

// Klasa edp_irp6ot_effector.
#include "robot/irp6ot_tfg/edp_irp6ot_tfg_effector.h"
#include "base/edp/reader.h"
// Klasa hardware_interface.
#include "robot/hi_moxa/hi_moxa.h"
// Klasa servo_buffer.
#include "robot/irp6ot_tfg/sg_irp6ot_tfg.h"
#include "robot/irp6ot_tfg/regulator_irp6ot_tfg.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_tfg {

#define CCM 1

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer(effector &_master) :
		common::servo_buffer(_master), master(_master)
{
	synchro_axis_order[0] = 0;
	axe_inc_per_revolution[0] = AXIS_7_INC_PER_REVOLUTION;
	synchro_step_coarse[0] = AXIS_7_SYNCHRO_STEP_COARSE;
	synchro_step_fine[0] = AXIS_7_SYNCHRO_STEP_FINE;

	thread_id = boost::thread(boost::bind(&servo_buffer::operator(), this));
}
/*-----------------------------------------------------------------------*/

void servo_buffer::load_hardware_interface(void)
{
	// tablica pradow maksymalnych dla poszczegolnych osi
	//int max_current[lib::irp6p_tfg::NUM_OF_SERVOS] = { AXIS_7_MAX_CURRENT };

	const std::vector <std::string> ports_vector(mrrocpp::lib::irp6ot_tfg::ports_strings, mrrocpp::lib::irp6ot_tfg::ports_strings
			+ mrrocpp::lib::irp6ot_tfg::LAST_MOXA_PORT_NUM + 1);
	hi =
			new hi_moxa::HI_moxa(master, mrrocpp::lib::irp6ot_tfg::LAST_MOXA_PORT_NUM, ports_vector, mrrocpp::lib::irp6ot_tfg::CARD_ADDRESSES, mrrocpp::lib::irp6ot_tfg::MAX_INCREMENT, mrrocpp::lib::irp6ot_tfg::TX_PREFIX_LEN);
	hi->init();

	//Ustawienie zwlocznego ograniczenia pradowego - dlugotrwale przekroczenie ustawionej wartosci
	//spowoduje wlaczenie stopu awaryjnego przez sterownik
	//	hi->set_parameter_now(0, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_tfg::MAX_CURRENT_0);
	hi->set_parameter_now(0, NF_COMMAND_SetDrivesMaxCurrent, mrrocpp::lib::irp6ot_tfg::MAX_CURRENT_0);
	//hi->set_parameter_now(0, NF_COMMAND_SetDrivesMaxCurrent, mrrocpp::lib::conveyor::MAX_CURRENT_0);

#ifdef CCM
	hi->set_parameter_now(0, NF_COMMAND_SetDrivesMode, NF_DrivesMode_CURRENT);
#else
	hi->set_parameter_now(0, NF_COMMAND_SetDrivesMode, NF_DrivesMode_PWM);
#endif

	// utworzenie tablicy regulatorow
	// Serwomechanizm 1
	regulator_ptr[0] = new NL_regulator_8_irp6ot(0, 0, 0, 0.39, 8.62 / 2., 7.89 / 2., 0.35, master);

	common::servo_buffer::load_hardware_interface();
}

/*-----------------------------------------------------------------------*/
uint64_t servo_buffer::compute_all_set_values(void)
{
	// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
	uint64_t status = OK; // kumuluje numer bledu

	for (int j = 0; j < master.number_of_servos; j++) {
		if (master.robot_test_mode) {
			regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step() * axe_inc_per_revolution[j]
					/ (2 * M_PI));
		} else {
			regulator_ptr[j]->insert_measured_current(hi->get_current(j));
			regulator_ptr[j]->insert_new_pos_increment(hi->get_increment(j));
		}
		// obliczenie nowej wartosci zadanej dla napedu
		status |= ((uint64_t) regulator_ptr[j]->compute_set_value()) << 2 * j;
		// przepisanie obliczonej wartosci zadanej do hardware interface
#ifdef CCM
		hi->set_current(j, regulator_ptr[j]->get_set_value());
	//	std::cout << "des current: " << regulator_ptr[j]->get_set_value() << std::endl;
#else
		hi->set_pwm(j, regulator_ptr[j]->get_set_value());
#endif
	}
	return status;
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions(void)
{
	common::servo_buffer::get_all_positions();

	// przepisanie stanu regulatora chwytaka do bufora odpowiedzi dla EDP_master
	servo_data.gripper_reg_state = regulator_ptr[0]->get_reg_state();

}
/*-----------------------------------------------------------------------*/

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp
