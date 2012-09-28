#include <iostream>
#include <exception>

#include "edp_typedefs.h"
#include "edp_e_manip.h"
#include "base/lib/mis_fun.h"
#include "reader.h"
#include "base/kinematics/kinematic_model_with_tool.h"
#include "edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

void force::operator()()
//!< watek do komunikacji ze sprzetem
{
	//	sr_msg->message("operator");

	if (!master.robot_test_mode) {
		lib::set_thread_priority(lib::PTHREAD_MAX_PRIORITY - 1);
	}

	try {
		if (!force_sensor_test_mode) {
			connect_to_hardware();
		}

		thread_started.command();

		configure_sensor();
	}

	catch (lib::exception::se_sensor & error) {
		std::cerr << "sensor_error w force thread EDP" << std::endl;

		uint64_t error0 = 0;

		if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
			error0 = *tmp;
		}

		switch (error0)
		{
			case SENSOR_NOT_CONFIGURED:
				from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
				break;
		}
		sr_msg->message(lib::FATAL_ERROR, error0);

	}

	catch (std::exception & e) {
		printf("force sensor exception: %s\n", e.what());
		sr_msg->message(lib::FATAL_ERROR, e.what());
		exit(EXIT_SUCCESS);
	}

	catch (...) {
		std::cerr << "unidentified error force thread w EDP" << std::endl;
	}

	if (clock_gettime(CLOCK_MONOTONIC, &wake_time) == -1) {
		perror("clock_gettime()");
	}

	while (!boost::this_thread::interruption_requested()) {
		try {
			if (new_edp_command) {
				boost::mutex::scoped_lock lock(mtx);
				// TODO: this should be handled with boost::bind functor parameter
				switch (command)
				{
					case (common::FORCE_SET_TOOL):
						set_force_tool();
						break;
					case (common::FORCE_CONFIGURE):
						configure_sensor();
						break;
					default:
						break;
				}
				set_command_execution_finish();
			} else {

				//	sr_msg->message("else 12");
				wait_for_event();

				get_reading();

				first_measure_synchroniser.command();
			}

		} //!< koniec TRY

		catch (lib::exception::se_sensor & error) {
			std::cerr << "sensor_error w force thread EDP" << std::endl;

			uint64_t error0 = 0;

			if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
				error0 = *tmp;
			}

			switch (error0)
			{
				case SENSOR_NOT_CONFIGURED:
					from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
					break;
				case READING_NOT_READY:
					from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
					break;
			}
			sr_msg->message(lib::FATAL_ERROR, error0);

		}

		catch (...) {
			std::cerr << "unidentified error in EDP force thread" << std::endl;
		}
	}
} //!< end MAIN

force::force(common::manip_effector &_master) :
		force_sensor_test_mode(true),
		is_reading_ready(false), //!< nie ma zadnego gotowego odczytu
		is_right_turn_frame(true),
		gravity_transformation(NULL),
		master(_master),
		is_sensor_configured(false),
		new_edp_command(false),
		cb(FORCE_BUFFER_LENGHT) //!< czujnik niezainicjowany
{
	/*! Lokalizacja procesu wywietlania komunikatow SR */

	sr_msg =
			boost::shared_ptr <lib::sr_vsp>(new lib::sr_vsp(lib::EDP, "f_" + master.config.robot_name, master.config.get_sr_attach_point()));

	sr_msg->message("force");

	if (master.config.exists(lib::FORCE_SENSOR_TEST_MODE)) {
		force_sensor_test_mode = master.config.exists_and_true(lib::FORCE_SENSOR_TEST_MODE);
	}

	if (force_sensor_test_mode) {
		sr_msg->message("Force sensor test mode activated");
	}

	if (master.config.exists("is_right_turn_frame")) {
		is_right_turn_frame = master.config.value <bool>("is_right_turn_frame");
	}

	for (int i = 0; i < 6; ++i) {
		ft_table[i] = 0.0;
	}

	clear_cb();

}

void force::clear_cb()
{

	lib::Ft_vector zero_force;
	for (int i = 0; i < 6; ++i) {
		zero_force[i] = 0.0;
	}
	cb.clear();

	for (int i = 0; i < FORCE_BUFFER_LENGHT; i++) {
		cb.push_back(zero_force);
	}

}

void force::wait_for_event()
{
	if (!force_sensor_test_mode) {

		wait_for_particular_event();

	} else {
		usleep(1000);
	}
}

/***************************** odczyt z czujnika *****************************/
void force::get_reading(void)
{
	/*
	 static int i = 1;

	 if (((i++) % 1000) == 0) {
	 std::cout << "force sensor:" << i << std::endl;
	 }
	 */
	if (!is_sensor_configured) {
		BOOST_THROW_EXCEPTION(lib::exception::fe_sensor() << lib::exception::mrrocpp_error0(SENSOR_NOT_CONFIGURED));
	}

	is_reading_ready = true;
	// if force_test_mode is not set read the force from sensor. Otherwise it remains zero.
	if (!force_sensor_test_mode) {
		get_particular_reading();
	}

	lib::Homog_matrix current_frame = master.servo_current_frame_wo_tool_dp.read();
	lib::Homog_matrix current_rotation(current_frame.return_with_with_removed_translation());
	// przypsieszenie we wszystkich osiach w ukladzie imu z watku imu
	lib::Xyz_Angle_Axis_vector imu_acc = master.imu_acc_dp.read();

	lib::Ft_vector force_output;

	// sprawdzenie ograniczen na sile
	bool overforce = false;
	for (int i = 0; i < 6; i++) {
		if ((fabs(ft_table[i]) > force_constraints[i]) || (!(std::isfinite(ft_table[i])))) {
			overforce = true;

		}
	}

	if (!overforce) {
		//sily przechowujemy w zerowej orientacji bazowej w ukladzie nadgarstka
		lib::Ft_vector base_force = gravity_transformation->getForce(ft_table, current_rotation);

		// dodanie nowej sily do bufora dla celow usredniania (filtracji dolnoprzepustowej)
		cb.push_back(base_force);

		// usredniamy za FORCE_BUFFER_LENGHT pomiarow
		for (int j = 0; j < 6; j++) {
			force_output[j] = 0.0;
		}

		for (int i = 0; i < FORCE_BUFFER_LENGHT; i++) {

			for (int j = 0; j < 6; j++) {
				force_output[j] += (cb[i][j]) / (double) FORCE_BUFFER_LENGHT;
			}

		}
		//zapis do bufora wymiany z watkiem transformation
		master.force_dp.write(force_output);

	} else {
		// wypisanie komunikatu o przekroczeniu sily progowej
		std::stringstream buffer(std::stringstream::in | std::stringstream::out);
		buffer << "over_force detected step: " << master.step_counter << " ";
		for (int i = 0; i < 6; i++) {
			buffer << i << ": " << ft_table[i] << " ";
		}

		sr_msg->message(lib::NON_FATAL_ERROR, buffer.str());
	}

	// przygotowanie odczytu dla readera przetransformowanego do ukladu narzedzia

	lib::Ft_tr ft_tr_inv_current_rotation_matrix(!current_rotation);

	lib::Homog_matrix current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*) master.get_current_kinematic_model())->tool);
	lib::Ft_tr ft_tr_inv_tool_matrix(!current_tool);

	lib::Ft_vector current_force_in_tool(ft_tr_inv_tool_matrix * ft_tr_inv_current_rotation_matrix * force_output);

	// scope-locked reader data update
	{
		if (master.rb_obj) {
			boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

			current_force_in_tool.to_table(master.rb_obj->step_data.force);
			imu_acc.to_table(master.rb_obj->step_data.real_cartesian_acc);
		} else {
			//	std::cerr << " " << std::endl;
		}
	}

}

/**************************** inicjacja czujnika ****************************/
void force::configure_sensor(void)
{ // by Y

	is_sensor_configured = true;
	//  printf("edp Sensor configured\n");
	sr_msg->message("edp Sensor configured");

	if (!force_sensor_test_mode) {
		configure_particular_sensor();
	}

	clear_cb();

	// polozenie kisci bez narzedzia wzgledem bazy
	lib::Homog_matrix current_frame = master.servo_current_frame_wo_tool_dp.read(); // FORCE Transformation by Slawomir Bazant

	if (!gravity_transformation) // nie powolano jeszcze obiektu
	{

		// zczytanie sil maksymalnych
		if (master.config.exists("force_constraints")) {
			char *tmp = strdup(master.config.value <std::string>("force_constraints").c_str());
			char* toDel = tmp;
			for (int i = 0; i < 6; i++) {
				force_constraints[i] = strtod(tmp, &tmp);
			}
			free(toDel);
		}

		//zczytanie polozenia czujnika wzgledem nadgarstka
		lib::Xyz_Angle_Axis_vector tab;
		if (master.config.exists("sensor_in_wrist")) {
			char *tmp = strdup(master.config.value <std::string>("sensor_in_wrist").c_str());
			char* toDel = tmp;
			for (int i = 0; i < 6; i++) {
				tab[i] = strtod(tmp, &tmp);
			}
			sensor_frame = lib::Homog_matrix(tab);
			free(toDel);

		}

		// zczytanie ciezaru narzedzia
		double weight = master.config.value <double>("weight");

		// polzoenie sredka ciezkosci narzedzia wzgledem nadgarstka
		double point[3];
		char *tmp = strdup(master.config.value <std::string>("default_mass_center_in_wrist").c_str());
		char* toDel = tmp;
		for (int i = 0; i < 3; i++)
			point[i] = strtod(tmp, &tmp);
		free(toDel);

		lib::K_vector pointofgravity(point);
		gravity_transformation =
				new lib::ForceTrans(force_sensor_name, current_frame, sensor_frame, weight, pointofgravity, is_right_turn_frame);
	} else {
		gravity_transformation->synchro(current_frame);
	}

}

force::~force()
{
}

void force::set_force_tool(void)
{
	lib::K_vector gravity_arm_in_sensor(next_force_tool_position);
	lib::Homog_matrix frame = master.servo_current_frame_wo_tool_dp.read();
	gravity_transformation->defineTool(frame, next_force_tool_weight, gravity_arm_in_sensor);

	current_force_tool_position = next_force_tool_position;
	current_force_tool_weight = next_force_tool_weight;
}

void force::set_command_execution_finish() // podniesienie semafora
{
	new_edp_command = false;
	new_command_synchroniser.command();
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
