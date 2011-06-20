/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */
#include "ui_robot.h"
#include "interface.h"
#include "base/ecp/ecp_robot.h"
/* TR
 #include "ui/src/wnd_base.h"
 */
#include "base/lib/messip/messip_dataport.h"

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

#include "wgt_robot_process_control.h"
#include "menu_bar.h"
#include "mainwindow.h"
#include "menu_bar_action.h"
#include "signal_dispatcher.h"

namespace mrrocpp {
namespace ui {
namespace common {

//
//
// KLASA UiRobot
//
//

UiRobot::UiRobot(Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos) :
	interface(_interface), tid(NULL), eb(_interface), robot_name(_robot_name), number_of_servos(_number_of_servos)
{
	//activation_string = _activation_string;
	state.edp.section_name = interface.config->get_edp_section(robot_name);
	state.ecp.section_name = interface.config->get_ecp_section(robot_name);
	state.edp.state = -1; // edp nieaktywne
	state.edp.last_state = -2; // edp nieokreslone
	state.ecp.trigger_fd = lib::invalid_fd;
	state.edp.is_synchronised = false; // edp nieaktywne
	msg	= (boost::shared_ptr <lib::sr_ecp>) new lib::sr_ecp(lib::ECP, "ui_" + robot_name, interface.network_sr_attach_point);

	process_control_window_created = false;
	wgt_robot_pc = 0L;
}

UiRobot::~UiRobot()
{
	delete wgt_robot_pc;
}

void UiRobot::create_thread()
{
	//	assert(tid == NULL);
	if (!tid) {
		tid = new feb_thread(eb);
	}
}


//wgt_base* UiRobot::getWgtByName(QString name)
//{
////	wgt_finder = ;
////	(*wgts_finder).second;
//	if((*wgts.find(name)).second==NULL)
//		printf("(*wgts.find(name)).second NULL!");
//	else
//		printf("(*wgts.find(name)).second ok");
//
//	return (*wgts.find(name)).second;
//}

void UiRobot::setup_menubar()
{
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	EDP_Load = new Ui::MenuBarAction(QString("EDP &Load"), this, menuBar);
	EDP_Unload = new Ui::MenuBarAction(QString("EDP &Unload"), this, menuBar);
	wgt_robot_process_control_action = new Ui::MenuBarAction(QString("Process &control"), this, menuBar);

	robot_menu = new QMenu(menuBar->menuRobot);
	robot_menu->setEnabled(true);

	robot_menu->addAction(EDP_Load);
	robot_menu->addAction(EDP_Unload);
	robot_menu->addAction(wgt_robot_process_control_action);

	robot_menu->addSeparator();
	menuBar->menuRobot->addAction(robot_menu->menuAction());

	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	connect(EDP_Load, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_EDP_Load_triggered(mrrocpp::ui::common::UiRobot*)), 	Qt::AutoCompatConnection);
	connect(EDP_Unload, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_EDP_Unload_triggered(mrrocpp::ui::common::UiRobot*)),	Qt::AutoCompatConnection);
	connect(wgt_robot_process_control_action, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_robot_process_control_triggered(mrrocpp::ui::common::UiRobot*)),	Qt::AutoCompatConnection);
}

bool UiRobot::is_process_control_window_created()
{
	return process_control_window_created;
}

void UiRobot::indicate_process_control_window_creation()
{
	process_control_window_created = true;
}

void UiRobot::block_ecp_trigger()
{
	if(wgt_robot_pc)
		wgt_robot_pc->block_all_ecp_trigger_widgets();
}

void UiRobot::unblock_ecp_trigger()
{
	if(wgt_robot_pc)
		wgt_robot_pc->unblock_all_ecp_trigger_widgets();
}

void UiRobot::set_robot_process_control_window(wgt_robot_process_control *wgt_pc)
{
	wgt_robot_pc = wgt_pc;
	if(interface.get_wgt_pc()->isVisible())
		wgt_robot_pc->my_open();
}

void UiRobot::open_robot_process_control_window()
{
	if(wgt_robot_pc)
		wgt_robot_pc->my_open();
}

void UiRobot::delete_robot_process_control_window()
{
	if(wgt_robot_pc)
		wgt_robot_pc->my_close();
	delete wgt_robot_pc;
	wgt_robot_pc = NULL;
}

wgt_robot_process_control * UiRobot::get_wgt_robot_pc()
{
	return wgt_robot_pc;
}

int UiRobot::edp_create_int()

{

	interface.set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota bird_hand
		if (state.edp.state == 0) {

			state.edp.is_synchronised = false;

			if (interface.check_node_existence(state.edp.node_name, robot_name)) {

				state.edp.node_nr = interface.config->return_node_number(state.edp.node_name);
				{
					boost::unique_lock <boost::mutex> lock(interface.process_creation_mtx);
					try {
						create_ui_ecp_robot();
					}

					catch (ecp::common::robot::ECP_main_error & e) {
						/* Obsluga bledow ECP */
						null_ui_ecp_robot();
						throw ecp::common::robot::ECP_main_error(e.error_class, e.error_no);

					} /*end: catch */
				}

				try {
					state.edp.pid = ui_get_edp_pid();

					if (state.edp.pid < 0) {

						state.edp.state = 0;
						fprintf(stderr, "edp spawn failed: %s\n", strerror(errno));
						delete_ui_ecp_robot();
					} else { // jesli spawn sie powiodl

						state.edp.state = 1;

						connect_to_reader();

						// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
						lib::controller_state_t robot_controller_initial_state_tmp;

						ui_get_controler_state(robot_controller_initial_state_tmp);

						//state.edp.state = 1; // edp wlaczone reader czeka na start

						state.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
					}
				}

				catch (ecp::common::robot::ECP_main_error & e) {
					/* Obsluga bledow ECP */
					close_edp_connections();
					null_ui_ecp_robot();

				} /*end: catch */

			}
		}

	} // end try

	catch (ecp::common::robot::ECP_main_error & e) {
		/* Obsluga bledow ECP */

	} /*end: catch */

	catch (ecp::common::robot::ECP_error & er) {
		/* Wylapywanie bledow generowanych przez modul transmisji danych do EDP */
		catch_ecp_error(er);
	} /* end: catch */

	catch (const std::exception & e) {
		catch_std_exception(e);
	}

	catch (...) { /* Dla zewnetrznej petli try*/
		/* Wylapywanie niezdefiniowanych bledow*/
		catch_tridot();
	} /*end: catch */

	interface.manage_interface();
	edp_create_int_extra_operations();

	return 1;

}

const lib::robot_name_t UiRobot::getName()
{
	return robot_name;
}

void UiRobot::close_all_windows()
{

	BOOST_FOREACH(const common::WndBase_pair_t & window_node, wndbase_m)
				{
					window_node.second->close();
				}

}

void UiRobot::abort_thread()
{
	assert(tid);
	delete tid;
	tid = NULL;
}

void UiRobot::connect_to_reader()
{
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia

	while ((state.edp.reader_fd = messip::port_connect(state.edp.network_reader_attach_point)) == lib::invalid_fd) {
		if ((tmp++) < lib::CONNECT_RETRY) {
			usleep(lib::CONNECT_DELAY);
		} else {
			perror("blad odwolania do READER");
			break;
		}
	}
}

bool UiRobot::pulse_reader_start_exec_pulse()
{
	if (state.edp.state == 1) {
		pulse_reader_execute(READER_START, 0);
		state.edp.state = 2;
		return true;
	}

	return false;
}

bool UiRobot::pulse_reader_stop_exec_pulse()
{
	if (state.edp.state == 2) {
		pulse_reader_execute(READER_STOP, 0);
		state.edp.state = 1;
		return true;
	}

	return false;
}

bool UiRobot::pulse_reader_trigger_exec_pulse()
{
	if (state.edp.state == 2) {
		pulse_reader_execute(READER_TRIGGER, 0);

		return true;
	}

	return false;
}

void UiRobot::pulse_reader_execute(int code, int value)
{

	if (messip::port_send_pulse(state.edp.reader_fd, code, value)) {
		perror("Blad w wysylaniu pulsu do redera");
	}
}

void UiRobot::connect_to_ecp_pulse_chanell()
{
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia
	// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem

	while ((state.ecp.trigger_fd = messip::port_connect(state.ecp.network_trigger_attach_point)) == NULL

	) {
		if (errno == EINTR)
			break;
		if ((tmp++) < lib::CONNECT_RETRY) {
			usleep(lib::CONNECT_DELAY);
		} else {
			perror("blad odwolania do ECP_TRIGGER");
		}
	}

	/*
	 // odwolanie alarmu
	 ualarm((useconds_t)(0), 0);
	 */
}

void UiRobot::pulse_ecp_execute(int code, int value)
{

	if (messip::port_send_pulse(state.ecp.trigger_fd, code, value))

	{
		fprintf(stderr, "Blad w wysylaniu pulsu do ecp error: %s \n", strerror(errno));
		delay(1000);
	}
}

void UiRobot::edp_create()
{
	if (state.edp.state == 0) {
		create_thread();

		eb.command(boost::bind(&ui::common::UiRobot::edp_create_int, &(*this)));
	}
}

int UiRobot::edp_create_int_extra_operations()
{
	return 1;
}

void UiRobot::pulse_ecp()
{
	if (state.edp.is_synchronised) { // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (state.ecp.trigger_fd == lib::invalid_fd) {
			connect_to_ecp_pulse_chanell();
		}

		if (state.ecp.trigger_fd != lib::invalid_fd) {
			pulse_ecp_execute(ECP_TRIGGER, 1);
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}
}

bool UiRobot::deactivate_ecp_trigger()
{

	if (state.is_active) {
		if (state.ecp.trigger_fd != lib::invalid_fd) {

			if (messip::port_disconnect(state.ecp.trigger_fd) != 0) {
				fprintf(stderr, "UIRobot::ECP trigger @%s:%d: messip::port_disconnect(): %s\n", __FILE__, __LINE__, strerror(errno));
			}

		}
		state.ecp.trigger_fd = lib::invalid_fd;
		state.ecp.pid = -1;
		return true;
	}

	return false;
}

void UiRobot::close_edp_connections()
{

	if (state.edp.reader_fd != lib::invalid_fd) {

		if (messip::port_disconnect(state.edp.reader_fd) != 0) {
			fprintf(stderr, "UIRobot::EDP_slay_int@%s:%d: messip::port_disconnect(): %s\n", __FILE__, __LINE__, strerror(errno));
		}

	}
	state.edp.reader_fd = lib::invalid_fd;

	close_all_windows();

	delete_ui_ecp_robot();

	state.edp.state = 0; // edp wylaczone
	state.edp.is_synchronised = false;

	state.edp.pid = -1;
}

void UiRobot::EDP_slay_int()
{
	// dla robota bird_hand
	if (state.edp.state > 0) { // jesli istnieje EDP

		close_edp_connections();

		interface.wait_for_child_termiantion((pid_t) state.edp.pid);

		abort_thread();
	}

	// modyfikacja menu

	interface.manage_interface();
}

// ustala stan wszytkich EDP

bool UiRobot::check_synchronised_and_loaded()
{
	return (((state.edp.state > 0) && (state.edp.is_synchronised)));

}

int UiRobot::move_to_synchro_position()
{
	return 1;
}

int UiRobot::move_to_front_position()
{
	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{
	return 1;
}

int UiRobot::reload_configuration()
{

	//	printf("final_position: %lf, %lf, %lf, %lf, %lf, %lf\n ", final_position[0], final_position[1], final_position[2], final_position[3], final_position[4], final_position[5]);

	// jesli IRP6 on_track ma byc aktywne
	if ((state.is_active = interface.config->exists_and_true("is_active", state.edp.section_name)) == 1) {
		// ini_con->create_ecp_irp6_on_track (ini_con->ui->ECP_SECTION);
		//ui_state.is_any_edp_active = true;
		if (interface.is_mp_and_ecps_active) {
			state.ecp.network_trigger_attach_point = interface.config->get_ecp_trigger_attach_point(robot_name);

			state.ecp.pid = -1;
			state.ecp.trigger_fd = lib::invalid_fd;
		}

		switch (state.edp.state)
		{
			case -1:
			case 0:
				// ini_con->create_edp_irp6_on_track (ini_con->ui->EDP_SECTION);

				state.edp.pid = -1;
				state.edp.reader_fd = lib::invalid_fd;
				state.edp.state = 0;

				for (int i = 0; i < 4; i++) {
					char tmp_string[50];
					if (i < 3) {
						sprintf(tmp_string, "preset_position_%d", i);
					} else {
						sprintf(tmp_string, "front_position");
					}

					if (interface.config->exists(tmp_string, state.edp.section_name)) {

						std::string text(interface.config->value <std::string> (tmp_string, state.edp.section_name));

						boost::char_separator <char> sep(" ");
						boost::tokenizer <boost::char_separator <char> > tokens(text, sep);

						int j = 0;
						BOOST_FOREACH(std::string t, tokens)
									{

										if (i < 3) {
											//value = boost::lexical_cast<double>(my_string);

											state.edp.preset_position[i][j] = boost::lexical_cast <double>(t);
										} else {
											state.edp.front_position[j] = boost::lexical_cast <double>(t);
										}

										if (j == number_of_servos) {
											break;
										}
										j++;
									}

					} else {
						for (int j = 0; j < number_of_servos; j++) {
							if (i < 3) {
								state.edp.preset_position[i][j] = 0.0;
							} else {
								state.edp.front_position[j] = 0.0;
								printf("nie zdefiniowano front_position w common.ini\n");
							}

						}
					}
				}

				if (interface.config->exists(lib::ROBOT_TEST_MODE, state.edp.section_name))
					state.edp.test_mode = interface.config->value <int> (lib::ROBOT_TEST_MODE, state.edp.section_name);
				else
					state.edp.test_mode = 0;

				state.edp.hardware_busy_attach_point = interface.config->get_edp_hardware_busy_file(robot_name);

				state.edp.network_resourceman_attach_point
						= interface.config->get_edp_resourceman_attach_point(robot_name);

				state.edp.network_reader_attach_point = interface.config->get_edp_reader_attach_point(robot_name);

				if (!interface.config->exists("node_name", state.edp.section_name)) {
					state.edp.node_name = "localhost";
				} else {
					state.edp.node_name = interface.config->value <std::string> ("node_name", state.edp.section_name);
				}
				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}

	} else // jesli  irp6 on_track ma byc nieaktywne
	{
		switch (state.edp.state)
		{
			case -1:
			case 0:
				state.edp.state = -1;
				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}
	} // end irp6_on_track

	return 1;
}

void UiRobot::catch_ecp_main_error(ecp::common::robot::ECP_main_error & e)

{
	if (e.error_class == lib::SYSTEM_ERROR)
		printf("ecp lib::SYSTEM_ERROR error in UI\n");
	interface.ui_state = 2;
}

void UiRobot::catch_ecp_error(ecp::common::robot::ECP_error & er)

{
	if (er.error_class == lib::SYSTEM_ERROR) { /* blad systemowy juz wyslano komunikat do SR */
		perror("ecp lib::SYSTEM_ERROR in UI");
		/* PtExit( EXIT_SUCCESS ); */
	} else {
		switch (er.error_no)
		{
			case INVALID_POSE_SPECIFICATION:
			case INVALID_COMMAND_TO_EDP:
			case EDP_ERROR:
			case INVALID_ROBOT_MODEL_TYPE:
				/* Komunikat o bledzie wysylamy do SR */
				msg->message(lib::NON_FATAL_ERROR, er.error_no);
				break;
			default:
				msg->message(lib::NON_FATAL_ERROR, 0, "ecp: Unidentified exception");
				perror("Unidentified exception");
		} /* end: switch */
	}
}

void UiRobot::catch_std_exception(const std::exception & e)
{

	std::string tmp_string(" The following error has been detected: ");
	tmp_string += e.what();
	msg->message(lib::NON_FATAL_ERROR, tmp_string.c_str());
	std::cerr << "UI: The following error has been detected :\n\t" << e.what() << std::endl;
}

void UiRobot::catch_tridot()

{
	/* Wylapywanie niezdefiniowanych bledow*/
	/* Komunikat o bledzie wysylamy do SR (?) */
	fprintf(stderr, "unidentified error in UI\n");
}

}
} //namespace ui
} //namespace mrrocpp
