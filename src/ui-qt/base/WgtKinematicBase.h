#ifndef WgtKinematicBase_H_
#define WgtKinematicBase_H_

#include "wgt_base.h"
#include <QGridLayout>
#include <QPushButton>
#include <iostream>
#include "ui_wgt_kinematic_template.h"

class WgtKinematicBase : public wgt_base
{
Q_OBJECT

public:
	WgtKinematicBase(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent =
			0);
	~WgtKinematicBase();

	const static int desired_pos_column = 5;
	const static int inc_move_column = 7;

protected:
	Ui::wgt_absolute_template ui;
	QVector <QDoubleSpinBox*> current_pos_spin_boxes;
	QVector <QLabel*> axis_labels;
	QPushButton *copy_button;

	void create_buttons_and_spin_boxes();
	void synchro_depended_widgets_disable(bool set_disabled);
	virtual void setup_ui(QGridLayout *layout, int _rows_number);
	virtual void add_current_position_spin_box(QDoubleSpinBox *spin_box, int row);

private:
	void create_buttons();

	int copy();
	virtual void move_it()
	{
	}
	virtual void init()
	{
	}

public slots:

	void on_pushButton_set_clicked();
	void on_pushButton_read_clicked();
	void copy_button_clicked();
	void init_and_copy_slot();
};

#endif /* WgtKinematicBase_H_ */
