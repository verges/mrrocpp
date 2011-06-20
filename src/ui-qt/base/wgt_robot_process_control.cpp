#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include "../irp6p_m/ui_r_irp6p_m.h"
#include "../conveyor/ui_r_conveyor.h"

#include "wgt_robot_process_control.h"
#include "interface.h"
#include "ui_robot.h"

wgt_robot_process_control::wgt_robot_process_control(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
	wgt_base(QString::fromStdString(robo->getName()), _interface, parent), ui(new Ui::wgt_robot_process_controlClass), robot(robo)
{
	ui->setupUi(this);
	ui->robot_label->setText(QString::fromStdString((robo->getName())));
	connect(this, SIGNAL(process_control_window_init_signal()), this, SLOT(process_control_window_init_slot()), Qt::QueuedConnection);
}

wgt_robot_process_control::~wgt_robot_process_control()
{
	delete ui;
}

Ui::wgt_robot_process_controlClass * wgt_robot_process_control::get_ui()
{
	return ui;
}

void wgt_robot_process_control::process_control_window_init()
{
	emit process_control_window_init_signal();
}

void wgt_robot_process_control::process_control_window_init_slot()
{
	init();
}

void wgt_robot_process_control::my_open(bool set_on_top)
{
	wgt_base::my_open(set_on_top);
	process_control_window_init();
}


//ECP
void wgt_robot_process_control::on_ecp_trigger_pushButton_clicked()
{
	interface.pulse_trigger_ecp(robot);
}

// Reader
void wgt_robot_process_control::on_reader_start_pushButton_clicked()
{
	interface.pulse_start_reader(robot);
}

void wgt_robot_process_control::on_reader_stop_pushButton_clicked()
{
	interface.pulse_stop_reader(robot);
}

void wgt_robot_process_control::on_reader_trigger_pushButton_clicked()
{
	interface.pulse_trigger_reader(robot);
}

// aktualizacja ustawien przyciskow
int wgt_robot_process_control::init()

{

	bool wlacz_PtButton_wnd_processes_control_all_reader_start = false;
	bool wlacz_PtButton_wnd_processes_control_all_reader_stop = false;
	bool wlacz_PtButton_wnd_processes_control_all_reader_trigger = false;

	// Dla READER'A


	ui->reader_start_pushButton->setDisabled(true);
	ui->reader_stop_pushButton->setDisabled(true);
	ui->reader_trigger_pushButton->setDisabled(true);

	// Dla irp6_on_track


	robot->process_control_window_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);
//	interface.irp6ot_m->process_control_window_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	// Dla irp6_postument

//	interface.irp6p_m->process_control_window_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	// Dla conveyor

//	interface.conveyor->process_control_window_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	// All reader's pulse buttons
	if (wlacz_PtButton_wnd_processes_control_all_reader_start) {
		ui->reader_start_pushButton->setDisabled(false);

	}

	if (wlacz_PtButton_wnd_processes_control_all_reader_stop) {
		ui->reader_stop_pushButton->setDisabled(false);

	}

	if (wlacz_PtButton_wnd_processes_control_all_reader_trigger) {
		ui->reader_trigger_pushButton->setDisabled(false);

	}

	// Dla mp i ecp
	if (interface.mp.state != interface.mp.last_process_control_state) {
		switch (interface.mp.state)
		{
			case ui::common::UI_MP_NOT_PERMITED_TO_RUN:
			case ui::common::UI_MP_PERMITED_TO_RUN:
				block_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_WAITING_FOR_START_PULSE:
				block_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_TASK_RUNNING:
				unblock_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_TASK_PAUSED:

				block_all_ecp_trigger_widgets();
				break;
			default:

				break;
		}

		interface.mp.last_process_control_state = interface.mp.state;

	}

	return 1;

}

int wgt_robot_process_control::block_all_ecp_trigger_widgets()

{

	/* TR

	 if (interface.irp6ot_m->state.edp.is_synchronised) {
	 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_ecp_trigger);
	 }
	 if (interface.irp6p_m->state.edp.is_synchronised) {
	 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_ecp_trigger);
	 }
	 if (interface.conveyor->state.edp.is_synchronised) {
	 interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_ecp_trigger);
	 }
	 */

	ui->ecp_trigger_pushButton->setDisabled(true);

	return 1;
}

int wgt_robot_process_control::unblock_all_ecp_trigger_widgets()

{

	/* TR

	 if (interface.irp6ot_m->state.edp.is_synchronised) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_ecp_trigger);
	 }
	 if (interface.irp6p_m->state.edp.is_synchronised) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_ecp_trigger);
	 }
	 if (interface.conveyor->state.edp.is_synchronised) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_conveyor_ecp_trigger);
	 }
	 */

	ui->ecp_trigger_pushButton->setDisabled(false);
	return 1;
}
