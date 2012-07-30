/*!
 * @file
 * @brief File containing the declaration of the RYS IMU sensor class.
 *
 * @author yoyek
 *
 */

#if !defined(_EDP_S_RYS_IMU_H)
#define _EDP_S_RYS_IMU_H


#include "base/edp/edp_imu_sensor.h"
#include "imu.hpp"

namespace mrrocpp {
namespace edp {
namespace sensor {




class rys_imu : public imu
{
public:

	rys_imu(common::manip_effector &_master);
	virtual ~rys_imu();

	void connect_to_hardware(void);
	void disconnect_from_hardware(void);
	void configure_particular_sensor(void);
	void wait_for_particular_event(void);
	void get_particular_reading(void);

private:
	IMU* ri;
};

} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif

