/*
 * WgtKinematicBase.cpp
 *
 *  Created on: 14-07-2011
 *      Author: matt
 */

#include "WgtKinematicBase.h"
#include "ui_robot.h"
#include "interface.h"
#include "mainwindow.h"

WgtKinematicBase::WgtKinematicBase(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		wgt_base(_widget_label, _interface, robo, parent)
{

}

WgtKinematicBase::~WgtKinematicBase()
{
	// TODO Auto-generated destructor stub
}

void WgtKinematicBase::setup_ui(QGridLayout *layout, int _rows_number)
{
	wgt_base::setup_ui(layout, _rows_number);

	create_buttons_and_spin_boxes();

	for (int i = 0; i < rows_number; i++) {
		gridLayout->addWidget(create_label_to_vector(axis_labels), i + 1, 0, 1, 1);
	}

	create_buttons();

	wgt_base::create_buttons_and_spin_boxes(desired_pos_column, inc_move_column, rows_number);
}

void WgtKinematicBase::create_buttons_and_spin_boxes()
{
	for (int i = 0; i < rows_number; i++)
		add_current_position_spin_box(create_spin_box_to_vector(current_pos_spin_boxes), i);
}

void WgtKinematicBase::add_current_position_spin_box(QDoubleSpinBox *spin_box, int row)
{
	gridLayout->addWidget(spin_box, row + 1, 1, 1, 1);
}

void WgtKinematicBase::create_buttons()
{

	copy_button = add_button(">", 1, 3, rows_number, 1);
	connect(copy_button, SIGNAL(clicked()), this, SLOT(copy_button_clicked()), Qt::QueuedConnection);

}

void WgtKinematicBase::synchro_depended_widgets_disable(bool set_disabled)
{
	copy_button->setDisabled(set_disabled);
	ui.pushButton_set->setDisabled(set_disabled);
	ui.pushButton_read->setDisabled(set_disabled);

	for (int i = 0; i < rows_number; i++) {
		current_pos_spin_boxes[i]->setDisabled(set_disabled);
		desired_pos_spin_boxes[i]->setDisabled(set_disabled);
	}

}

void WgtKinematicBase::on_pushButton_read_clicked()
{
	printf("read\n");
	init();
}

void WgtKinematicBase::init_and_copy_slot()
{
	init();
	copy();
}

void WgtKinematicBase::copy_button_clicked()
{
	copy();
}

int WgtKinematicBase::copy()
{

	if (robot->state.edp.pid != -1) {
		if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_set->setDisabled(false);
			for (int i = 0; i < rows_number; i++) {
				desired_pos_spin_boxes[i]->setValue(current_pos_spin_boxes[i]->value());
			}
		} else
			ui.pushButton_set->setDisabled(true); // Wygaszanie elementow przy niezsynchronizowanym robocie
	}

	return 1;
}

void WgtKinematicBase::on_pushButton_set_clicked()
{
	get_desired_position();
	move_it();
}

