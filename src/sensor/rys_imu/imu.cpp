#include "imu.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>

using boost::lexical_cast;

IMU::IMU(const std::string& port, int baud) {
	connected = false;
	
	gyro_factor = 14.375;
	acc_factor = 256;
	
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0) {
		tcgetattr(fd, &oldtio);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) {
			fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
			tcsetattr(fd, TCSANOW, &oldtio);
			close(fd);
			fd = -1;
			return;
		}
		// activate new settings
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);
		connected = true;
	}
}

IMU::~IMU() {
	// restore old port settings
	if (fd > 0)
		tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
}

void IMU::setFactors(double gf, double af) {
	gyro_factor = gf;
	acc_factor = af;
}

ImuData IMU::getReading() {
	ImuData ret;
	std::string buf;
	int retcode;
	char c = ' ';
	
	// ignore everything before start character
	while (c != '$')
		read(fd, &c, 1);
		
	// remember everything up to end character
	while (c != '#') {
		retcode = read(fd, &c, 1);
		if (retcode == 0)
			continue;
			
		if (retcode < 0)
			throw "Can't read data from serial port!";
			
		if (c != '#')
			buf += c;
	}

	tcflush(fd, TCIFLUSH);

	std::vector<std::string> strs;
	boost::split(strs, buf, boost::is_any_of(","));
	
	if (strs.size() != 6) 
		throw "Bad measurements count!";
	
	for (int i = 0; i < 3; ++i) {
		ret.angularVelocity[i] = lexical_cast<double>(strs[i+3]) / gyro_factor;
		ret.linearAcceleration[i] = lexical_cast<double>(strs[i]) / acc_factor;
	}
	
	//std::cout << "Ang:\n" << ret.angularVelocity << "\nLin:\n" << ret.linearAcceleration << "\n";
	
	return ret;
}
