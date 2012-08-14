/*!
 * @file
 * @brief File containing methods of the rys imu sensor class.
 *
 * @author yoyek
 *
 */

#include <cstdio>
#include <exception>
#include <ctime>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mis_fun.h"

#include "base/lib/sr/srlib.h"
#include "edp_s.h"

#include "base/edp/edp_e_manip.h"
// Konfigurator
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

rys_imu::rys_imu(common::manip_effector &_master) :
		imu(_master)
{
	printf("rys_imu created !!! \n");
	sr_msg->message("RYS IMU sensor being created");

}

void rys_imu::connect_to_hardware(void)
{
	ri = new IMU();
}

rys_imu::~rys_imu(void)
{
	if (!imu_sensor_test_mode) {
		disconnect_from_hardware();
	}
	printf("Destruktor rys sensor\n");
}

void rys_imu::disconnect_from_hardware(void)
{
	delete ri;
}

void rys_imu::configure_particular_sensor(void)
{
	sr_msg->message("RYS IMU sensor being configured");
}

void rys_imu::wait_for_particular_event()
{
	usleep(12000);
}

void rys_imu::get_particular_reading(void)
{
	static int i = 1;

	ldata = ri->getReading();

	if (((i++) % 1000) == 0) {
		std::cout << i << std::endl;
	}

//	std::cout << "Ang:\n" << ldata.angularVelocity << "\nLin:\n" << ldata.linearAcceleration << "\n";
}

imu* return_created_edp_imu_sensor(common::manip_effector &_master)
{
	return new rys_imu(_master);
}

/*****************************  *****************************/

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

