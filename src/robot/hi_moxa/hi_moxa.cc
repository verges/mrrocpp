/*
 * hi_moxa.cc
 *
 *  Created on: Nov 14, 2011
 *      Author: mwalecki
 */

#include <exception>
#include <stdexcept>
#include <cstring>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

#include "base/lib/periodic_timer.h"
#include "robot/hi_moxa/hi_moxa.h"
#include "base/edp/edp_e_motor_driven.h"

NF_STRUCT_ComBuf	NFComBuf;

namespace mrrocpp {
namespace edp {
namespace hi_moxa {


HI_moxa::HI_moxa(common::motor_driven_effector &_master, int last_drive_n, std::vector <std::string> ports, const double* max_increments) :
		common::HardwareInterface(_master),
		last_drive_number(last_drive_n),
		port_names(ports),
		ridiculous_increment(max_increments),
		ptimer(COMMCYCLE_TIME_NS / 1000000)
{
#ifdef T_INFO_FUNC
	std::cout << "[func] Hi, Moxa!" << std::endl;
#endif
}

HI_moxa::~HI_moxa()
{
#ifdef T_INFO_FUNC
	std::cout << "[func] Bye, Moxa!" << std::endl;
#endif
	for (unsigned int i = 0; i <= last_drive_number; i++) {
		if (fd[i] > 0) {
			tcsetattr(fd[i], TCSANOW, &oldtio[i]);
			close(fd[i]);
		}
	}
}

void HI_moxa::init()
{

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::init()" << std::endl;
#endif
	// inicjalizacja crcTable[]
	crcInit();


	// inicjalizacja zmiennych
	NFComBuf.myAddress = NF_TerminalAddress;

	for (unsigned int drive_number = 0; drive_number <= last_drive_number; drive_number++) {

		// TODO: Ta linia to ZUO!!!!!
		drives_addresses[drive_number] = 0x10 + drive_number;

		NFComBuf.ReadDeviceStatus.addr[drive_number] = drives_addresses[drive_number];
		NFComBuf.ReadDeviceVitals.addr[drive_number] = drives_addresses[drive_number];
		NFComBuf.SetDrivesMode.addr[drive_number] = drives_addresses[drive_number];
		NFComBuf.SetDrivesPWM.addr[drive_number] = drives_addresses[drive_number];
		NFComBuf.SetDrivesMaxCurrent.addr[drive_number] = drives_addresses[drive_number];
		NFComBuf.ReadDrivesCurrent.addr[drive_number] = drives_addresses[drive_number];
		NFComBuf.ReadDrivesPosition.addr[drive_number] = drives_addresses[drive_number];
		NFComBuf.SetDrivesMisc.addr[drive_number] = drives_addresses[drive_number];
		NFComBuf.ReadDrivesStatus.addr[drive_number] = drives_addresses[drive_number];

		NFComBuf.ReadDeviceStatus.data[drive_number] = 0;
		NFComBuf.ReadDeviceVitals.data[drive_number] = 0;
		NFComBuf.SetDrivesMode.data[drive_number] = 0;
		NFComBuf.SetDrivesPWM.data[drive_number] = 0;
		NFComBuf.SetDrivesMaxCurrent.data[drive_number] = 0;
		NFComBuf.ReadDrivesCurrent.data[drive_number] = 0;
		NFComBuf.ReadDrivesPosition.data[drive_number] = 0;
		NFComBuf.SetDrivesMisc.data[drive_number] = 0;
		NFComBuf.ReadDrivesStatus.data[drive_number] = 0;

		servo_data[drive_number].first_hardware_reads = FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT;
		servo_data[drive_number].command_params = 0;
		servo_data[drive_number].drive_status.sw1 = 0;
		servo_data[drive_number].drive_status.sw2 = 0;
		servo_data[drive_number].drive_status.swSynchr = 0;
		servo_data[drive_number].drive_status.synchroZero = 0;
		servo_data[drive_number].drive_status.powerStageFault = 0;
		servo_data[drive_number].drive_status.overcurrent = 0;
		servo_data[drive_number].drive_status.error = 0;
		servo_data[drive_number].drive_status.isSynchronized = 0;
		servo_data[drive_number].drive_status.current = 0;
		servo_data[drive_number].drive_status.position = 0;
		for (int j = 0; j < SERVO_ST_BUF_LEN; j++)
			servo_data[drive_number].buf[j] = 0;
	}
	hardware_panic = false;

	// informacja o stanie robota
	master.controller_state_edp_buf.is_power_on = true;
	master.controller_state_edp_buf.robot_in_fault_state = false;

	if (master.robot_test_mode) {
		// domyslnie robot jest zsynchronizowany
		master.controller_state_edp_buf.is_synchronised = true;
		// informacja o stanie robota
		master.controller_state_edp_buf.is_power_on = true;
		master.controller_state_edp_buf.robot_in_fault_state = false;
	} // end test mode
	else {
		// domyslnie robot nie jest zsynchronizowany
		master.controller_state_edp_buf.is_synchronised = false;

		fd_max = 0;
		for (unsigned int drive_number = 0; drive_number <= last_drive_number; drive_number++) {
			std::cout << "[info] opening port : " << port_names[drive_number].c_str();

			SerialPort[drive_number] = new SerialComm(port_names[drive_number].c_str(), BAUD);
			if(SerialPort[drive_number]->isConnected()){
				std::cout << "...OK" << std::endl;
			}
			else {
				std::cout << std::endl << "[error] Nie wykryto sprzetu!" << std::endl;
				throw(std::runtime_error("unable to open device!!!"));
			}

			// start driver in MANUAL mode
			set_parameter_now(drive_number, NF_COMMAND_SetDrivesMode, NF_DrivesMode_MANUAL);
		}
	}

	reset_counters();
}

void HI_moxa::set_pwm(int drive_number, double set_value)
{
	NFComBuf.SetDrivesPWM.data[drive_number] = set_value * (1000.0 / 255.0);
	commandArray[commandCnt++] = NF_COMMAND_SetDrivesPWM;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::set_pwm(" << drive_number << ", " << set_value << ")" << std::endl;
#endif
}

void HI_moxa::set_current(int drive_number, double set_value)
{
	NFComBuf.SetDrivesCurrent.data[drive_number] = (int)set_value * (1000.0 / 255.0);
	commandArray[commandCnt++] = NF_COMMAND_SetDrivesCurrent;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::set_current(" << drive_number << ", " << set_value << ")" << std::endl;
#endif
}

void HI_moxa::set_parameter(int drive_number, const int parameter, uint32_t new_value){
	switch (parameter)
	{
		case NF_COMMAND_SetDrivesMisc:
			NFComBuf.SetDrivesMisc.data[drive_number] = (uint32_t) new_value;
			commandArray[commandCnt++] = NF_COMMAND_SetDrivesMisc;
			break;
		case NF_COMMAND_SetDrivesMaxCurrent:
			NFComBuf.SetDrivesMaxCurrent.data[drive_number] = (int16_t) new_value;
			commandArray[commandCnt++] = NF_COMMAND_SetDrivesMaxCurrent;
			break;
		case NF_COMMAND_SetDrivesMode:
			NFComBuf.SetDrivesMode.data[drive_number] = (uint8_t) new_value;
			commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
			break;
		default:
			std::cout << "[error] HI_moxa::set_parameter() invalid parameter" << std::endl;
			return;
			break;
	}
}

int HI_moxa::get_current(int drive_number)
{
	int ret;
	ret = servo_data[drive_number].drive_status.current;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_current(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
}

float HI_moxa::get_voltage(int drive_number)
{
	float ret = VOLTAGE;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_voltage(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
}

double HI_moxa::get_increment(int drive_number)
{
	double ret;
	ret = servo_data[drive_number].current_position_inc;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_increment(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
}

long int HI_moxa::get_position(int drive_number)
{
	int ret;

	ret = servo_data[drive_number].current_absolute_position;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_position(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
}

uint64_t HI_moxa::read_write_hardware(void)
{
	static int64_t receive_attempts = 0;
	// UNUSED: static int64_t receive_timeouts = 0;
	static int error_msg_power_stage = 0;
	static int error_msg_hardware_panic = 0;
	static int error_msg_overcurrent = 0;
	static int last_synchro_state[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	static int comm_timeouts[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	static int synchro_switch_filter[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	const int synchro_switch_filter_th = 2;
	bool robot_synchronized = true;
	bool power_fault;
	bool hardware_read_ok = true;
	bool all_hardware_read = true;
	std::size_t bytes_received[MOXA_SERVOS_NR];
	uint64_t ret = 0;
	uint8_t drive_number;
	static int status_disp_cnt = 0;

	// test mode
	if (master.robot_test_mode) {
		ptimer.sleep();
		return ret;
	} // end test mode

	// If Hardware Panic, send PARAM_DRIVER_MODE_ERROR to motor drivers
	if (hardware_panic) {
		for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
			// only set error parameter, do not wait for answer
			commandCnt = 0;
			NFComBuf.SetDrivesMode.data[drive_number] = NF_DrivesMode_ERROR;
			commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
			txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf+5, (const uint8_t*)commandArray, commandCnt, drives_addresses[drive_number]);
			// Clear communication request
			commandCnt = 0;
			// Send command frame
			SerialPort[drive_number]->write(txBuf, txCnt+5);

		}
		if (error_msg_hardware_panic == 0) {
			master.msg->message(lib::FATAL_ERROR, "Hardware panic");
			std::cout << "[error] hardware panic" << std::endl;
			error_msg_hardware_panic++;
		}
		ptimer.sleep();
		return ret;
	} else {
		// Make command frames and send them to drives
		for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
			// Set communication requests
			commandArray[commandCnt++] = NF_COMMAND_ReadDrivesPosition;
			commandArray[commandCnt++] = NF_COMMAND_ReadDrivesCurrent;
			commandArray[commandCnt++] = NF_COMMAND_ReadDrivesStatus;
			// Make command frame
			txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf+5, (const uint8_t*)commandArray, commandCnt, drives_addresses[drive_number]);

			#ifdef NFV2_TX_DEBUG
			std::cout << "[debug] commandArray: ";
			for(int k=0; k<commandCnt; k++)
				std::cout << (unsigned int)commandArray[k] << ";";
			std::cout << std::endl;
			std::cout << "[debug] txBuf: ";
			for(int k=0; k<txCnt; k++)
				std::cout << (unsigned int)txBuf[k+5] << ";";
			std::cout << std::endl;
			#endif //NFV2_DEBUG

			// Clear communication requests
			commandCnt = 0;
			// Send command frame
			SerialPort[drive_number]->write(txBuf, txCnt+5);
		}
	}

	receive_attempts++;

	struct timespec delay;
	delay.tv_nsec = 700000;
	delay.tv_sec = 0;

	nanosleep(&delay, NULL);

	// Tu kiedys byl SELECT

	all_hardware_read = true;
	// Read data from all drives
	for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
		rxCnt = 0;
		while(1){
			#ifdef NFV2_RX_DEBUG
			uint8_t maxRxCnt = 0;
			#endif //NFV2_DEBUG
			if(SerialPort[drive_number]->read(&(rxBuf[rxCnt]), 1) > 0 && (rxCnt < 255)){
				#ifdef NFV2_RX_DEBUG
				maxRxCnt = (rxCnt > maxRxCnt) ? rxCnt : maxRxCnt;
				#endif //NFV2_DEBUG
				if(NF_Interpreter(&NFComBuf, rxBuf, &rxCnt, rxCommandArray, &rxCommandCnt) > 0){
					// TODO: Check Status
					#ifdef NFV2_RX_DEBUG
					std::cout << "[debug] rxBuf: ";
					for(int k=0; k<=maxRxCnt; k++)
						std::cout << (unsigned int)rxBuf[k] << ";";
					std::cout << std::endl;
					#endif //NFV2_DEBUG
					break;
				}
			}
			else{
				comm_timeouts[drive_number]++;
				if (all_hardware_read) {
					all_hardware_read = false;
					std::cout << "[error] timeout in " << (int) receive_attempts << " communication cycle on drives";
				}
				std::cout << " " << (int) drive_number << "(" << port_names[drive_number].c_str() << ")" << std::endl;
				break;
			}
		}
	}

	// If Hardware Panic, after receiving data, wait till the end of comm cycle and return.
	if (hardware_panic) {
		ptimer.sleep();
		return ret;
	}

	if (all_hardware_read) {
		for (drive_number = 0; drive_number <= last_drive_number; drive_number++)
			comm_timeouts[drive_number] = 0;
	} else {
		std::cout << std::endl;
		hardware_read_ok = false;
	}

	// Inicjalizacja flag
	robot_synchronized = true;
	power_fault = false;

	for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {

		// Wypelnienie pol odebranymi danymi
		if (bytes_received[drive_number] >= READ_BYTES) {
			servo_data[drive_number].previous_absolute_position = servo_data[drive_number].current_absolute_position;
			servo_data[drive_number].current_absolute_position = NFComBuf.ReadDrivesPosition.data[drive_number];
			//std::cout << "Current Absolute Position = " << servo_data[drive_number].current_absolute_position << std::endl;
		}

		// Ustawienie flagi wlaczonej mocy
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_PowerStageFault) != 0) {
			power_fault = true;
		}

		// Ustawienie flagi synchronizacji
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_Synchronized) == 0) {
			robot_synchronized = false;
		}

		// Sprawdzenie, czy wlasnie nastapila synchronizacja kolejnej osi
		if (last_synchro_state[drive_number] == 0
					&& (NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_Synchronized) != 0) {
			servo_data[drive_number].first_hardware_reads = FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT;
			last_synchro_state[drive_number] = 1;
		}

		// W pierwszych odczytach danych z napedu przyrost pozycji musi byc 0.
		if ((servo_data[drive_number].first_hardware_reads > 0)) {
			servo_data[drive_number].previous_absolute_position = servo_data[drive_number].current_absolute_position;
			servo_data[drive_number].first_hardware_reads--;
		}

		// Sprawdzenie przyrostu pozycji enkodera
		servo_data[drive_number].current_position_inc = (double) (servo_data[drive_number].current_absolute_position
				- servo_data[drive_number].previous_absolute_position);

		if ((robot_synchronized) && ((int)ridiculous_increment[drive_number] != 0)) {
			if ((servo_data[drive_number].current_position_inc > ridiculous_increment[drive_number])
					|| (servo_data[drive_number].current_position_inc < -ridiculous_increment[drive_number])) {
				hardware_panic = true;
				std::stringstream temp_message;
				temp_message << "[error] ridiculous increment on drive " << (int) drive_number << ", "
						<< port_names[drive_number].c_str() << ", c.cycle " << (int) receive_attempts << ": read = "
						<< servo_data[drive_number].current_position_inc << ", max = "
						<< ridiculous_increment[drive_number] << std::endl;
				master.msg->message(lib::FATAL_ERROR, temp_message.str());
				std::cout << temp_message.str();
			}
		}

		// Sprawdzenie ograniczenia nadpradowego
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_Overcurrent) != 0) {
			if (error_msg_overcurrent == 0) {
				master.msg->message(lib::NON_FATAL_ERROR, "Overcurrent");
				std::cout << "[error] overcurrent on drive " << (int) drive_number << ", "
						<< port_names[drive_number].c_str() << ": read = "
						<< servo_data[drive_number].drive_status.current << "mA" << std::endl;
				error_msg_overcurrent++;
			}
		}

		// Wykrywanie sekwencji timeoutow komunikacji
		if (comm_timeouts[drive_number] >= MAX_COMM_TIMEOUTS) {
			hardware_panic = true;
			std::stringstream temp_message;
			temp_message << "[error] multiple communication timeouts on drive " << (int) drive_number << "("
					<< port_names[drive_number].c_str() << "): limit = " << MAX_COMM_TIMEOUTS << std::endl;
			master.msg->message(lib::FATAL_ERROR, temp_message.str());
			std::cout << temp_message.str();
		}

	}

	master.controller_state_edp_buf.is_synchronised = robot_synchronized;
	master.controller_state_edp_buf.robot_in_fault_state = power_fault;
	if (power_fault) {
		if (error_msg_power_stage == 0) {
			master.msg->message(lib::NON_FATAL_ERROR, "Wylaczono moc - robot zablokowany");
			error_msg_power_stage++;
		}

	} else {
		error_msg_power_stage = 0;
	}

	for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_LimitSwitchUp) != 0)
			ret |= (uint64_t) (UPPER_LIMIT_SWITCH << (5 * (drive_number))); // Zadzialal wylacznik "gorny" krancowy
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_LimitSwitchDown) != 0)
			ret |= (uint64_t) (LOWER_LIMIT_SWITCH << (5 * (drive_number))); // Zadzialal wylacznik "dolny" krancowy
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_EncoderIndexSignal) != 0)
			ret |= (uint64_t) (SYNCHRO_ZERO << (5 * (drive_number))); // Impuls zera rezolwera
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_Overcurrent) != 0)
			ret |= (uint64_t) (OVER_CURRENT << (5 * (drive_number))); // Przekroczenie dopuszczalnego pradu
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_SynchroSwitch) != 0) {
			if (synchro_switch_filter[drive_number] == synchro_switch_filter_th)
				ret |= (uint64_t) (SYNCHRO_SWITCH_ON << (5 * (drive_number))); // Zadzialal wylacznik synchronizacji
			else
				synchro_switch_filter[drive_number]++;
		} else {
			synchro_switch_filter[drive_number] = 0;
		}
	}

	if (status_disp_cnt++ == STATUS_DISP_T) {
		// UNUSED: const int disp_drv_no = 0;
		//		std::cout << "[info]";
		//		std::cout << " sw1_sw2_swSynchr = " << (int) servo_data[disp_drv_no].drive_status.sw1 << "," << (int) servo_data[disp_drv_no].drive_status.sw2 << "," << (int) servo_data[disp_drv_no].drive_status.swSynchr;
		//		std::cout << " position = " << (int) servo_data[disp_drv_no].drive_status.position;
		//		std::cout << " current = " << (int) servo_data[disp_drv_no].drive_status.current;
		//		std::cout << std::endl;

		//		for(int disp_drv_no=0; disp_drv_no<6; disp_drv_no++)
		//		{
		//			std::cout << "   " << (int) servo_data[disp_drv_no].drive_status.sw1 << "," << (int) servo_data[disp_drv_no].drive_status.sw2 << "," << (int) servo_data[disp_drv_no].drive_status.swSynchr;
		//		}
		//
		//		if(servo_data[5].drive_status.swSynchr != 0)
		//			std::cout << "   ########################### ########################### ";
		//
		//		std::cout << std::endl;

		status_disp_cnt = 0;
	}

	ptimer.sleep();

	return ret;
}

int HI_moxa::set_parameter_now(int drive_number, const int parameter, uint32_t new_value)
{
	struct timespec delay;
	uint8_t setParamCommandCnt = 0;
	uint8_t setParamCommandArray[10];

	switch (parameter)
	{
		case NF_COMMAND_SetDrivesMisc:
			NFComBuf.SetDrivesMisc.data[drive_number] = (uint32_t) new_value;
			setParamCommandArray[setParamCommandCnt++] = NF_COMMAND_SetDrivesMisc;
			break;
		case NF_COMMAND_SetDrivesMaxCurrent:
			NFComBuf.SetDrivesMaxCurrent.data[drive_number] = (int16_t) new_value;
			setParamCommandArray[setParamCommandCnt++] = NF_COMMAND_SetDrivesMaxCurrent;
			break;
		case NF_COMMAND_SetDrivesMode:
			NFComBuf.SetDrivesMode.data[drive_number] = (uint8_t) new_value;
			setParamCommandArray[setParamCommandCnt++] = NF_COMMAND_SetDrivesMode;
			break;
		default:
			std::cout << "[error] HI_moxa::set_parameter_now() invalid parameter" << std::endl;
			return -1;
			break;
	}

	// Add Read Drive Status request
	setParamCommandArray[setParamCommandCnt++] = NF_COMMAND_ReadDrivesStatus;
	//commandArray[commandCnt++] = NF_COMMAND_ReadDeviceVitals;
	// Make command frame
	txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf+5, (const uint8_t*)setParamCommandArray, setParamCommandCnt, drives_addresses[drive_number]);
	// Clear communication request
	setParamCommandCnt = 0;

	for (int param_set_attempt = 0; param_set_attempt < MAX_PARAM_SET_ATTEMPTS; param_set_attempt++) {
		// Send command frame
		SerialPort[drive_number]->write(txBuf, txCnt+5);

		// hardware panic; do not print error information; do not wait for response
		if (parameter == NF_COMMAND_SetDrivesMode && new_value == NF_DrivesMode_ERROR)
			return 0;

		// Give some time for a response to return
		delay.tv_nsec = 500000;
		delay.tv_sec = 0;
		nanosleep(&delay, NULL);

		rxCnt = 0;
		while(1){
			if(SerialPort[drive_number]->read(&(rxBuf[rxCnt]), 1) > 0 && (rxCnt < 255)){
				if(NF_Interpreter(&NFComBuf, rxBuf, &rxCnt, rxCommandArray, &rxCommandCnt) > 0){
					// TODO: Check status;
					return 0;
				}
			}
			else {
				std::cout << "[error] param set ack timeout for drive (" << drive_number << ")" << std::endl;
				break;
			}
		}
	}
	throw std::runtime_error("HI_Moxa: Unable to communicate with motor controllers. Try switching the power on.");
	return 1;
}

void HI_moxa::reset_counters(void)
{

	for (int i = 0; i < master.number_of_servos; i++) {

		servo_data[i].current_absolute_position = 0L;
		servo_data[i].previous_absolute_position = 0L;
		servo_data[i].current_position_inc = 0.0;

	} // end: for

	//	std::cout << "[func] HI_moxa::reset_counters" << std::endl;
}

void HI_moxa::start_synchro(int drive_number)
{
	NFComBuf.SetDrivesMode.data[drive_number] = NF_DrivesMode_SYNC_PWM0;
	commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::start_synchro(" << drive_number << ")" << std::endl;
	//#endif
}

void HI_moxa::finish_synchro(int drive_number)
{
	NFComBuf.SetDrivesMode.data[drive_number] = NF_DrivesMode_PWM;
	commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::finish_synchro(" << drive_number << ")" << std::endl;
	//#endif
}

bool HI_moxa::in_synchro_area(int drive_number)
{
	return ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_SynchroSwitch) != 0);
}

bool HI_moxa::robot_synchronized()
{
	bool ret = true;
	for (std::size_t drive_number = 0; drive_number <= last_drive_number; drive_number++) {
		if ((NFComBuf.ReadDrivesStatus.data[drive_number] & NF_DrivesStatus_SynchroSwitch) == 0) {
			ret = false;
		}
	}
	return ret;
}

bool HI_moxa::is_impulse_zero(int drive_number)
{
	std::cout << "[func] HI_moxa::is_impulse_zero(" << drive_number << ")" << std::endl;
	return false;
}

void HI_moxa::reset_position(int drive_number)
{
	servo_data[drive_number].current_absolute_position = 0L;
	servo_data[drive_number].previous_absolute_position = 0L;
	servo_data[drive_number].current_position_inc = 0.0;
	servo_data[drive_number].first_hardware_reads = FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT;
	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::reset_position(" << drive_number << ")" << std::endl;
	//#endif
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

