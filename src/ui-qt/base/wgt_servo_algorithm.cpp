/*
 * wgt_servo_algorithm.cpp
 *
 *  Created on: 14-07-2011
 *      Author: matt
 */

#include "wgt_servo_algorithm.h"
#include "mainwindow.h"
#include "ui_robot.h"
#include "interface.h"
#include "ui_r_common_012.h"
#include "allrobots.h"
#include "mp.h"
#include "ui_ecp_robot/ui_ecp_r_common_012.h"

wgt_servo_algorithm::wgt_servo_algorithm(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		wgt_base(_widget_label, _interface, robo, parent), ui(new Ui::wgt_servo_algorithm_templateClass)
{
	robot = dynamic_cast <mrrocpp::ui::common_012::UiRobot *>(robo);
	ui->setupUi(this);
	dwgt->setWindowTitle(QString::fromStdString((robo->getName())) + " sa");

	setup_ui(ui->grid_up, robot->number_of_servos);

	for (int i = 0; i < robot->number_of_servos; ++i) {
		axis_labels[i]->setText(QString("q%1").arg(i));
	}

}

wgt_servo_algorithm::~wgt_servo_algorithm()
{
	// TODO Auto-generated destructor stub
}

void wgt_servo_algorithm::setup_ui(QGridLayout *layout, int _rows_number)
{

	wgt_base::setup_ui(layout, _rows_number);

	create_buttons_and_spin_boxes();

	for (int i = 0; i < rows_number; i++) {
		gridLayout->addWidget(create_label_to_vector(axis_labels), i + 1, 0, 1, 1);
	}

	create_buttons();

}

void wgt_servo_algorithm::create_buttons_and_spin_boxes()
{
	for (int i = 0; i < rows_number; i++) {
		//	add_current_position_spin_box(create_spin_box_to_vector(current_pos_spin_boxes), i);
	}
}

void wgt_servo_algorithm::add_current_position_spin_box(QDoubleSpinBox *spin_box, int row)
{
	gridLayout->addWidget(spin_box, row + 1, 1, 1, 1);
}

void wgt_servo_algorithm::create_buttons()
{

	copy_button = add_button(">", 1, 4, rows_number, 1);
	connect(copy_button, SIGNAL(clicked()), this, SLOT(copy_button_clicked()), Qt::QueuedConnection);

}

void wgt_servo_algorithm::on_pushButton_read_clicked()
{
	printf("read\n");
	init();
}

void wgt_servo_algorithm::on_pushButton_set_clicked()
{
	printf("set\n");
}

void wgt_servo_algorithm::init()
{
}

void wgt_servo_algorithm::init_and_copy_slot()
{
	init();
	copy();
}

void wgt_servo_algorithm::copy_button_clicked()
{
	copy();
}

int wgt_servo_algorithm::copy()
{

	if (robot->state.edp.pid != -1) {

		for (int i = 0; i < rows_number; i++) {
			//		desired_servo_algorithm_boxes[i]->setValue(current_servo_algorithm_boxes[i]->value());
			//		desired_servo_parameters_boxes[i]->setValue(current_servo_parameters_boxes[i]->value());
		}
	}

	return 1;
}

