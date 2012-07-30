#ifndef IMU_HPP
#define IMU_HPP

#include <stdint.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <fstream>

#include "base/lib/imudata.hpp"

class IMU {
public:
	IMU(const std::string& port = "/dev/ttyUSB0", int baud = B57600);
	~IMU();

	// Block until new measurement is avalible
	ImuData getReading();	
	
	void setFactors(double gf, double af);

protected:

private:
	int fd;
	struct termios oldtio;
	
	bool connected;
	
	double gyro_factor;
	double acc_factor;
};

#endif /* IMU_HPP */
