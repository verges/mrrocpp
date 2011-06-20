/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_irp6ot_tfg.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/wgt_single_motor_move.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"




namespace mrrocpp {
namespace ui {
namespace irp6ot_tfg {
const std::string WGT_IRP6OT_TFG_MOVE = "WGT_IRP6OT_TFG_MOVE";
//
//
// KLASA UiRobot
//
//


int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->ecp->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);
}

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::common::EcpRobot(*this);
	return 1;
}

int UiRobot::edp_create_int_extra_operations()
{
	wgt_move->synchro_depended_init();
	return 1;
}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::irp6ot_tfg::UiRobot::synchronise_int, &(*this)));

	return 1;

}

int UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::irp6ot_tfg::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_synchro_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = 0.0;
	}
	eb.command(boost::bind(&ui::irp6ot_tfg::UiRobot::execute_motor_motion, &(*this)));

	return 1;
}

int UiRobot::execute_motor_motion()
{
	try {

		ui_ecp_robot->move_motors(desired_pos);

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

int UiRobot::execute_joint_motion()
{
	try {

		ui_ecp_robot->move_joints(desired_pos);

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

int UiRobot::synchronise_int()

{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6_on_track

		if ((state.edp.state > 0) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->ecp->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp irp6_on_track niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_IN_ROBOT

	// modyfikacje menu
	interface.manage_interface();
	wgt_move->synchro_depended_init();
	wgt_move->init_and_copy();
	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
	single_motor::UiRobot(_interface, lib::irp6ot_tfg::ROBOT_NAME, lib::irp6ot_tfg::NUM_OF_SERVOS)

{
	wgt_move = new wgt_single_motor_move("Irp6ot_tfg moves", interface, *this, interface.get_main_window());
	wndbase_m[WGT_IRP6OT_TFG_MOVE] = wgt_move->dwgt;
}

int UiRobot::manage_interface()
{

	MainWindow *mw = interface.get_main_window();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, robot_menu);

			break;
		case 0:
			mw->enable_menu_item(false, 3, EDP_Unload, actionirp6ot_tfg_Synchronization, actionirp6ot_tfg_Move);
			mw->enable_menu_item(false, 1, menuirp6ot_tfg_Preset_Positions);
			mw->enable_menu_item(true, 1, robot_menu);
			mw->enable_menu_item(true, 1, EDP_Load);

			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, robot_menu);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, actionirp6ot_tfg_Synchronization);
				mw->enable_menu_item(true, 1, mw->getMenuBar()->menuall_Preset_Positions);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 2, EDP_Unload, actionirp6ot_tfg_Move);
						mw->enable_menu_item(true, 1, menuirp6ot_tfg_Preset_Positions);
						mw->enable_menu_item(false, 1, EDP_Load);
						block_ecp_trigger();
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 1, actionirp6ot_tfg_Move);
						mw->enable_menu_item(true, 1, menuirp6ot_tfg_Preset_Positions);
						mw->enable_menu_item(false, 2, EDP_Load, EDP_Unload);
						block_ecp_trigger();
						break;
					case common::UI_MP_TASK_RUNNING:
						unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 1, menuirp6ot_tfg_Preset_Positions);
						mw->enable_menu_item(false, 1, actionirp6ot_tfg_Move);
						block_ecp_trigger();
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 3, EDP_Unload, actionirp6ot_tfg_Synchronization, actionirp6ot_tfg_Move);
				mw->enable_menu_item(false, 1, EDP_Load);

			}
			break;
		default:
			break;
	}

	return 1;
}
void UiRobot::make_connections()
{
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	connect(actionirp6ot_tfg_Synchronization, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionirp6ot_tfg_Move, 				SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Move_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionirp6ot_tfg_Synchro_Position,	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionirp6ot_tfg_Position_0, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionirp6ot_tfg_Position_1,		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionirp6ot_tfg_Position_2, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
}

void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	actionirp6ot_tfg_Synchronization= new Ui::MenuBarAction(QString("&Synchronization"), this, menuBar);
	actionirp6ot_tfg_Move 			= new Ui::MenuBarAction(QString("&Move"), this, menuBar);
	actionirp6ot_tfg_Synchro_Position = new Ui::MenuBarAction(QString("&Synchro Position"), this, menuBar);
	actionirp6ot_tfg_Position_0 	= new Ui::MenuBarAction(QString("Position &0"), this, menuBar);
	actionirp6ot_tfg_Position_1 	= new Ui::MenuBarAction(QString("Position &1"), this, menuBar);
	actionirp6ot_tfg_Position_2 	= new Ui::MenuBarAction(QString("Position &2"), this, menuBar);

	menuirp6ot_tfg_Preset_Positions = new QMenu(robot_menu);
	robot_menu->addSeparator();
	robot_menu->addAction(actionirp6ot_tfg_Synchronization);
	robot_menu->addAction(actionirp6ot_tfg_Move);
	robot_menu->addAction(menuirp6ot_tfg_Preset_Positions->menuAction());
	menuirp6ot_tfg_Preset_Positions->addAction(actionirp6ot_tfg_Synchro_Position);
	menuirp6ot_tfg_Preset_Positions->addAction(actionirp6ot_tfg_Position_0);
	menuirp6ot_tfg_Preset_Positions->addAction(actionirp6ot_tfg_Position_1);
	menuirp6ot_tfg_Preset_Positions->addAction(actionirp6ot_tfg_Position_2);

    robot_menu->setTitle(QApplication::translate("MainWindow", "Irp6ot_t&Fg", 0, QApplication::UnicodeUTF8));
    menuirp6ot_tfg_Preset_Positions->setTitle(QApplication::translate("MainWindow", "&Preset positions", 0, QApplication::UnicodeUTF8));
    make_connections();
}



}
} //namespace ui
} //namespace mrrocpp

