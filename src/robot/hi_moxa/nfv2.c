#include "nfv2.h"

/*
* Funkcja do wrzucenia w obsluge przychodzacych danych.
* NFComBuf - wskaznik do bufora komunikacyjnego, w ktorej automatycznie beda umieszczane odebrane dane.
* rxBuf - wskaznik do tablicy, w ktorej w przerwaniu znak po znaku zapisywana jest odebrana wiadomosc.
* rxPt - wskaznik do indeksu pola w tablicy, na ktorym zostala zapisany zostal ostatnio odebrany bajt.
* commandArray - wskaznik do tablicy polecen, ktora jest automatycznie wypelniana w przypadku odebrania zapytan.
* commandCnt - wskaznik do zmiennej zawierajacej liczbe polecen w commandArray[].
* Funkcja zwraca 1, jesli zostala poprawnie odebrana ramka adresowana do naszego modulu, 0 w pozostalych przypadkach.
*/
uint8_t NF_Interpreter(	NF_STRUCT_ComBuf *NFComBuf,
						volatile uint8_t *rxBuf,
						volatile uint8_t *rxPt,
						volatile uint8_t *commandArray,
						volatile uint8_t *commandCnt){
	static uint8_t n;	
	uint8_t rxAddress, rxParamsCnt;
	uint8_t rxBufIter, combufDataIter, rxDataIter, dataBytesIter;
	uint8_t *dataPt, *u8TempPt;

	if((*rxPt) == 0 && rxBuf[0]!='#'){
		return 0;
	}
	else if((*rxPt) == 3){
		if(( (rxBuf[1] & 0xff) == ((~rxBuf[2]) & 0xff) ) && (rxBuf[1] > 3)){
			n = rxBuf[1];
			(*rxPt)++;
		}
		else{
			(*rxPt) = 0;
		}
		return 0;
	}
	else if((*rxPt) > 3 && (*rxPt) >= n){
		(*rxPt) = 0;
		// if CRC Fail
		if(crcFast(((const uint8_t*)rxBuf) + 3,  n-3) != rxBuf[n]){
			return 0;
		}
		// if Address Fail && not a Broadcast
		//else if(rxBuf[3] != NFComBuf->myAddress && rxBuf[3] != NF_BroadcastAddress){
		//	return 0;
		//}
		// if CRC OK && Address OK go to Command Interpreter
	}
	else{
		(*rxPt)++;
		return 0;
	}
	
	// ############	NF Command Interpreter
	rxAddress = rxBuf[3];
	rxBufIter = 4; // First command starts here
	*commandCnt = 0;
	while(rxBufIter < n){
		dataPt = rxBuf + rxBufIter+2;
		
		// ############### "WRITE" Type Commands
		// if  rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress
		//		Master sends data to me (slave)
		
		// ########	Drives
		// ####		Set Mode
		#ifdef NF_BUFSZ_SetDrivesMode
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesMode){
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesMode;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesMode)) {
						u8TempPt = &(NFComBuf->SetDrivesMode.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesMode){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetDrivesMode + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetDrivesMode.data[combufDataIter] = ((NF_STRUCT_SetDrivesMode*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetDrivesMode.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Speed
		#ifdef NF_BUFSZ_SetDrivesSpeed
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesSpeed){ 
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesSpeed;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesSpeed)) {
						u8TempPt = &(NFComBuf->SetDrivesSpeed.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesSpeed){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetDrivesSpeed + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetDrivesSpeed.data[combufDataIter] = ((NF_STRUCT_SetDrivesSpeed*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetDrivesSpeed.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Current
		#ifdef NF_BUFSZ_SetDrivesCurrent
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesCurrent){				
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesCurrent;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesCurrent)) {
						u8TempPt = &(NFComBuf->SetDrivesCurrent.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesCurrent){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetDrivesCurrent + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetDrivesCurrent.data[combufDataIter] = ((NF_STRUCT_SetDrivesCurrent*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetDrivesCurrent.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Position
		#ifdef NF_BUFSZ_SetDrivesPosition
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesPosition){			
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesPosition;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesPosition)) {
						u8TempPt = &(NFComBuf->SetDrivesPosition.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesPosition){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetDrivesPosition + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetDrivesPosition.data[combufDataIter] = ((NF_STRUCT_SetDrivesPosition*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetDrivesPosition.updated = 1;
				}
			}
			else
		#endif
		// ####		Set PWM
		#ifdef NF_BUFSZ_SetDrivesPWM
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesPWM){			
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesPWM;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesPWM)) {
						u8TempPt = &(NFComBuf->SetDrivesPWM.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesPWM){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetDrivesPWM + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetDrivesPWM.data[combufDataIter] = ((NF_STRUCT_SetDrivesPWM*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetDrivesPWM.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Max Current
		#ifdef NF_BUFSZ_SetDrivesMaxCurrent
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesMaxCurrent){				
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesMaxCurrent;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesMaxCurrent)) {
						u8TempPt = &(NFComBuf->SetDrivesMaxCurrent.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesMaxCurrent){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetDrivesMaxCurrent + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetDrivesMaxCurrent.data[combufDataIter] = ((NF_STRUCT_SetDrivesMaxCurrent*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetDrivesMaxCurrent.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Misc
		#ifdef NF_BUFSZ_SetDrivesMisc
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesMisc){				
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesMisc;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesMisc)) {
						u8TempPt = &(NFComBuf->SetDrivesMisc.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesMisc){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetDrivesMisc + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetDrivesMisc.data[combufDataIter] = ((NF_STRUCT_SetDrivesMisc*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetDrivesMisc.updated = 1;
				}
			}
			else
		#endif
		
		// ########	Servos
		// ####		Set Mode
		#ifdef NF_BUFSZ_SetServosMode
			if(rxBuf[rxBufIter] == NF_COMMAND_SetServosMode){		
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetServosMode;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetServosMode)) {
						u8TempPt = &(NFComBuf->SetServosMode.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetServosMode){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetServosMode + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetServosMode.data[combufDataIter] = ((NF_STRUCT_SetServosMode*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetServosMode.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Position
		#ifdef NF_BUFSZ_SetServosPosition
			if(rxBuf[rxBufIter] == NF_COMMAND_SetServosPosition){		
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetServosPosition;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetServosPosition)) {
						u8TempPt = &(NFComBuf->SetServosPosition.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetServosPosition){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetServosPosition + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetServosPosition.data[combufDataIter] = ((NF_STRUCT_SetServosPosition*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetServosPosition.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Speed
		#ifdef NF_BUFSZ_SetServosSpeed
			if(rxBuf[rxBufIter] == NF_COMMAND_SetServosSpeed){		
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetServosSpeed;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetServosSpeed)) {
						u8TempPt = &(NFComBuf->SetServosSpeed.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetServosSpeed){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetServosSpeed + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetServosSpeed.data[combufDataIter] = ((NF_STRUCT_SetServosSpeed*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetServosSpeed.updated = 1;
				}
			}
			else
		#endif
		
		// ########	Digital IO
		// ####		Set Outputs
		#ifdef NF_BUFSZ_SetDigitalOutputs
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDigitalOutputs){		
				if(rxAddress == NFComBuf->myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDigitalOutputs;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDigitalOutputs)) {
						u8TempPt = &(NFComBuf->SetDigitalOutputs.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDigitalOutputs){
							*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_SetDigitalOutputs + dataBytesIter);
							dataBytesIter++;
						}
						//NFComBuf->SetDigitalOutputs.data[combufDataIter] = ((NF_STRUCT_SetDigitalOutputs*)dataPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf->SetDigitalOutputs.updated = 1;
				}
			}
			else
		#endif
		
		// ############### "READ" Type Commands
		// if rxParamsCnt == 0 && rxAddress == NFComBuf->myAddress
		//		Master wants to acquire data from me (slave)
		// if rxParamsCnt > 0  && rxAddress == NFComBuf->xxx.addr[i]
		//		Slave returns data that I (master) asked for

		// ########	Drives
		// ####		Read Current
		#ifdef NF_BUFSZ_ReadDrivesCurrent
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDrivesCurrent){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDrivesCurrent;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf->myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDrivesCurrent;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDrivesCurrent)) {
						if(NFComBuf->ReadDrivesCurrent.addr[combufDataIter] == rxAddress) {
							u8TempPt = &(NFComBuf->ReadDrivesCurrent.data[combufDataIter]);
							dataBytesIter = 0;
							while(dataBytesIter < NF_DATABYTES_ReadDrivesCurrent){
								*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_ReadDrivesCurrent + dataBytesIter);
								dataBytesIter++;
							}
							//NFComBuf->ReadDrivesCurrent.data[combufDataIter] = ((NF_STRUCT_ReadDrivesCurrent*)dataPt)->data[rxDataIter];
							rxDataIter++;
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		// ####		Read Position
		#ifdef NF_BUFSZ_ReadDrivesPosition
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDrivesPosition){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDrivesPosition;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf->myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDrivesPosition;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDrivesPosition)) {
						if(NFComBuf->ReadDrivesPosition.addr[combufDataIter] == rxAddress) {
							u8TempPt = &(NFComBuf->ReadDrivesPosition.data[combufDataIter]);
							dataBytesIter = 0;
							while(dataBytesIter < NF_DATABYTES_ReadDrivesPosition){
								*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_ReadDrivesPosition + dataBytesIter);
								dataBytesIter++;
							}
							//NFComBuf->ReadDrivesPosition.data[combufDataIter] = ((NF_STRUCT_ReadDrivesPosition*)dataPt)->data[rxDataIter];
							rxDataIter++;
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		// ####		Read Status
		#ifdef NF_BUFSZ_ReadDrivesStatus
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDrivesStatus){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDrivesStatus;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf->myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDrivesStatus;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDrivesStatus)) {
						if(NFComBuf->ReadDrivesStatus.addr[combufDataIter] == rxAddress) {
							u8TempPt = &(NFComBuf->ReadDrivesStatus.data[combufDataIter]);
							dataBytesIter = 0;
							while(dataBytesIter < NF_DATABYTES_ReadDrivesStatus){
								*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_ReadDrivesStatus + dataBytesIter);
								dataBytesIter++;
							}
							//NFComBuf->ReadDrivesStatus.data[combufDataIter] = ((NF_STRUCT_ReadDrivesStatus*)dataPt)->data[rxDataIter];
							rxDataIter++;
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		
		// ########	Digital IO
		// ####		Read Inputs
		#ifdef NF_BUFSZ_ReadDigitalInputs
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDigitalInputs){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDigitalInputs;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf->myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDigitalInputs;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDigitalInputs)) {
						if(NFComBuf->ReadDigitalInputs.addr[combufDataIter] == rxAddress) {
							u8TempPt = &(NFComBuf->ReadDigitalInputs.data[combufDataIter]);
							dataBytesIter = 0;
							while(dataBytesIter < NF_DATABYTES_ReadDigitalInputs){
								*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_ReadDigitalInputs + dataBytesIter);
								dataBytesIter++;
							}
							//NFComBuf->ReadDigitalInputs.data[combufDataIter] = ((NF_STRUCT_ReadDigitalInputs*)dataPt)->data[rxDataIter];
							rxDataIter++;						
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		
		// ########	Analog IO
		// ####		Read Inputs
		#ifdef NF_BUFSZ_ReadAnalogInputs
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadAnalogInputs){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadAnalogInputs;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf->myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadAnalogInputs;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadAnalogInputs)) {
						if(NFComBuf->ReadAnalogInputs.addr[combufDataIter] == rxAddress) {
							u8TempPt = &(NFComBuf->ReadAnalogInputs.data[combufDataIter]);
							dataBytesIter = 0;
							while(dataBytesIter < NF_DATABYTES_ReadAnalogInputs){
								*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_ReadAnalogInputs + dataBytesIter);
								dataBytesIter++;
							}
							//NFComBuf->ReadAnalogInputs.data[combufDataIter] = ((NF_STRUCT_ReadAnalogInputs*)dataPt)->data[rxDataIter];
							rxDataIter++;						
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		
		// ########	Device
		// ####		Read Status
		#ifdef NF_BUFSZ_ReadDeviceStatus
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDeviceStatus){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDeviceStatus;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf->myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDeviceStatus;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDeviceStatus)) {
						if(NFComBuf->ReadDeviceStatus.addr[combufDataIter] == rxAddress) {
							u8TempPt = &(NFComBuf->ReadDeviceStatus.data[combufDataIter]);
							dataBytesIter = 0;
							while(dataBytesIter < NF_DATABYTES_ReadDeviceStatus){
								*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_ReadDeviceStatus + dataBytesIter);
								dataBytesIter++;
							}
							//NFComBuf->ReadDeviceStatus.data[combufDataIter] = ((NF_STRUCT_ReadDeviceStatus*)dataPt)->data[rxDataIter];
							rxDataIter++;						
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		// ####		Read Vitals
		#ifdef NF_BUFSZ_ReadDeviceVitals
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDeviceVitals){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDeviceVitals;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf->myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDeviceVitals;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDeviceVitals)) {
						if(NFComBuf->ReadDeviceVitals.addr[combufDataIter] == rxAddress) {
							u8TempPt = &(NFComBuf->ReadDeviceVitals.data[combufDataIter]);
							dataBytesIter = 0;
							while(dataBytesIter < NF_DATABYTES_ReadDeviceVitals){
								*(u8TempPt + dataBytesIter) = *(dataPt + rxDataIter*NF_DATABYTES_ReadDeviceVitals + dataBytesIter);
								dataBytesIter++;
							}
							//NFComBuf->ReadDeviceVitals.data[combufDataIter] = ((NF_STRUCT_ReadDeviceVitals*)dataPt)->data[rxDataIter];
							rxDataIter++;						
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
				NFComBuf->unknownCommandRec = 1;
			
		
		rxBufIter += rxBuf[rxBufIter+1]+2;
	}
	
	return rxBufIter;
}

/*
* Funkcja buduje ramke komunikatu na podstawie danych w buforze komunikacji oraz listy zadanych polecen.
* NFComBuf - wskaznik do bufora komunikacyjnego, w ktorej automatycznie beda umieszczane odebrane dane.
* txBuf - wskaznik do tablicy, w ktorej zostanie utworzona wiadomosc.
* commandArray - wskaznik do tablicy polecen, ktore maja byc zawarte w komunikacie.
* commandCnt - ilosc polecen w tablicy commandArray[].
* Funkcja zwraca dlugosc zbudowanego komunikatu, zapisanego w txBuf.
*/
uint8_t NF_MakeCommandFrame(NF_STRUCT_ComBuf *NFComBuf,
							uint8_t *txBuf,
							const uint8_t *commandArray,
							uint8_t commandCnt,
							uint8_t txAddress){
	uint8_t txBufIter, commandIter, txDataIter, combufDataIter, dataBytesIter;	
	
	uint8_t *dataPt;
	uint8_t *u8TempPt;
	 
	txBuf[0] = '#';
	txBuf[3] = txAddress;

	txBufIter=4;
	commandIter=0;
		
	while(commandIter < commandCnt){
	
		dataPt = txBuf+txBufIter+2;

		// ########	Drives
		// ####		Set Mode
		#ifdef NF_BUFSZ_SetDrivesMode
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesMode){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesMode) {
					if(NFComBuf->SetDrivesMode.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetDrivesMode.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesMode){
							*(dataPt + txDataIter*NF_DATABYTES_SetDrivesMode + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetDrivesMode*)dataPt)->data[txDataIter] = NFComBuf->SetDrivesMode.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesMode;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesMode;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Speed
		#ifdef NF_BUFSZ_SetDrivesSpeed
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesSpeed){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesSpeed) {
					if(NFComBuf->SetDrivesSpeed.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetDrivesSpeed.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesSpeed){
							*(dataPt + txDataIter*NF_DATABYTES_SetDrivesSpeed + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetDrivesSpeed*)dataPt)->data[txDataIter] = NFComBuf->SetDrivesSpeed.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesSpeed;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesSpeed;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Current
		#ifdef NF_BUFSZ_SetDrivesCurrent
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesCurrent){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesCurrent) {
					if(NFComBuf->SetDrivesCurrent.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetDrivesCurrent.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesCurrent){
							*(dataPt + txDataIter*NF_DATABYTES_SetDrivesCurrent + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetDrivesCurrent*)dataPt)->data[txDataIter] = NFComBuf->SetDrivesCurrent.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesCurrent;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesCurrent;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Position
		#ifdef NF_BUFSZ_SetDrivesPosition
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesPosition){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesPosition) {
					if(NFComBuf->SetDrivesPosition.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetDrivesPosition.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesPosition){
							*(dataPt + txDataIter*NF_DATABYTES_SetDrivesPosition + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetDrivesPosition*)dataPt)->data[txDataIter] = NFComBuf->SetDrivesPosition.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesPosition;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesPosition;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif	
		// ####		Set PWM
		#ifdef NF_BUFSZ_SetDrivesPWM
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesPWM){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesPWM) {
					if(NFComBuf->SetDrivesPWM.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetDrivesPWM.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesPWM){
							*(dataPt + txDataIter*NF_DATABYTES_SetDrivesPWM + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetDrivesPosition*)dataPt)->data[txDataIter] = NFComBuf->SetDrivesPosition.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesPWM;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesPWM;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif	
		// ####		Set Max Current
		#ifdef NF_BUFSZ_SetDrivesMaxCurrent
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesMaxCurrent){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesMaxCurrent) {
					if(NFComBuf->SetDrivesMaxCurrent.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetDrivesMaxCurrent.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesMaxCurrent){
							*(dataPt + txDataIter*NF_DATABYTES_SetDrivesMaxCurrent + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetDrivesPosition*)dataPt)->data[txDataIter] = NFComBuf->SetDrivesPosition.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesMaxCurrent;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesMaxCurrent;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif	
		// ####		Set Misc
		#ifdef NF_BUFSZ_SetDrivesMisc
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesMisc){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesMisc) {
					if(NFComBuf->SetDrivesMisc.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetDrivesMisc.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDrivesMisc){
							*(dataPt + txDataIter*NF_DATABYTES_SetDrivesMisc + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetDrivesPosition*)dataPt)->data[txDataIter] = NFComBuf->SetDrivesPosition.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesMisc;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesMisc;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif

		// ########	Servos
		// ####		Set Mode
		#ifdef NF_BUFSZ_SetServosMode
			if(commandArray[commandIter] == NF_COMMAND_SetServosMode){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetServosMode) {
					if(NFComBuf->SetServosMode.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetServosMode.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetServosMode){
							*(dataPt + txDataIter*NF_DATABYTES_SetServosMode + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetServosMode*)dataPt)->data[txDataIter] = NFComBuf->SetServosMode.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetServosModen;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetServosMode;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Position
		#ifdef NF_BUFSZ_SetServosPosition
			if(commandArray[commandIter] == NF_COMMAND_SetServosPosition){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetServosPosition) {
					if(NFComBuf->SetServosPosition.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetServosPosition.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetServosPosition){
							*(dataPt + txDataIter*NF_DATABYTES_SetServosPosition + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetServosPosition*)dataPt)->data[txDataIter] = NFComBuf->SetServosPosition.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetServosPosition;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetServosPosition;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Speed
		#ifdef NF_BUFSZ_SetServosSpeed
			if(commandArray[commandIter] == NF_COMMAND_SetServosSpeed){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetServosSpeed) {
					if(NFComBuf->SetServosSpeed.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetServosSpeed.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetServosSpeed){
							*(dataPt + txDataIter*NF_DATABYTES_SetServosSpeed + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetServosSpeed*)dataPt)->data[txDataIter] = NFComBuf->SetServosSpeed.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetServosSpeed;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetServosSpeed;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif	  

		// ########	Digital IO
		// ####		Set Outputs
		#ifdef NF_BUFSZ_SetDigitalOutputs
			if(commandArray[commandIter] == NF_COMMAND_SetDigitalOutputs){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDigitalOutputs) {
					if(NFComBuf->SetDigitalOutputs.addr[combufDataIter] == txAddress) {
						u8TempPt = &(NFComBuf->SetDigitalOutputs.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_SetDigitalOutputs){
							*(dataPt + txDataIter*NF_DATABYTES_SetDigitalOutputs + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						//((NF_STRUCT_SetDigitalOutputs*)dataPt)->data[txDataIter] = NFComBuf->SetDigitalOutputs.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDigitalOutputs;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDigitalOutputs;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif 
		
		// ############### "READ" Type Commands
		// if txAddress == NFComBuf->myAddress
		//		I (slave) return data that master asked for
		// if rxAddress == NFComBuf->xxx.addr[i]
		//		I (master) want to acquire data from slave
		
		// ########	Device
		// ####		Read Status
		#ifdef NF_BUFSZ_ReadDeviceStatus
			if(commandArray[commandIter] == NF_COMMAND_ReadDeviceStatus){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf->myAddress){					
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadDeviceStatus) {
						u8TempPt = &(NFComBuf->ReadDeviceStatus.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_ReadDeviceStatus){
							*(dataPt + txDataIter*NF_DATABYTES_ReadDeviceStatus + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						txDataIter++;						
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadDeviceStatus;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadDeviceStatus;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadDeviceStatus;
					txBuf[txBufIter+1] = 0;	
					txBufIter += 2;
					
				}
			}
			else
		#endif
		// ####		Read Vitals
		#ifdef NF_BUFSZ_ReadDeviceVitals
			if(commandArray[commandIter] == NF_COMMAND_ReadDeviceVitals){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf->myAddress){					
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadDeviceVitals) {
						u8TempPt = &(NFComBuf->ReadDeviceVitals.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_ReadDeviceVitals){
							*(dataPt + txDataIter*NF_DATABYTES_ReadDeviceVitals + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						txDataIter++;						
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadDeviceVitals;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadDeviceVitals;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadDeviceVitals;
					txBuf[txBufIter+1] = 0;	
					txBufIter += 2;
					
				}
			}
			else
		#endif
		
		// ########	Drives
		// ####		Read Current
		#ifdef NF_BUFSZ_ReadDrivesCurrent
			if(commandArray[commandIter] == NF_COMMAND_ReadDrivesCurrent){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf->myAddress){
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadDrivesCurrent) {
						u8TempPt = &(NFComBuf->ReadDrivesCurrent.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_ReadDrivesCurrent){
							*(dataPt + txDataIter*NF_DATABYTES_ReadDrivesCurrent + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						txDataIter++;
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadDrivesCurrent;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadDrivesCurrent;
					txBufIter += txBuf[txBufIter+1]+2;
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadDrivesCurrent;
					txBuf[txBufIter+1] = 0;
					txBufIter += 2;

				}
			}
			else
		#endif
		// ####		Read Position
		#ifdef NF_BUFSZ_ReadDrivesPosition
			if(commandArray[commandIter] == NF_COMMAND_ReadDrivesPosition){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf->myAddress){
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadDrivesPosition) {
						u8TempPt = &(NFComBuf->ReadDrivesPosition.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_ReadDrivesPosition){
							*(dataPt + txDataIter*NF_DATABYTES_ReadDrivesPosition + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						txDataIter++;
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadDrivesPosition;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadDrivesPosition;
					txBufIter += txBuf[txBufIter+1]+2;
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadDrivesPosition;
					txBuf[txBufIter+1] = 0;
					txBufIter += 2;

				}
			}
			else
		#endif
		// ####		Read Status
		#ifdef NF_BUFSZ_ReadDrivesStatus
			if(commandArray[commandIter] == NF_COMMAND_ReadDrivesStatus){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf->myAddress){					
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadDrivesStatus) {
						u8TempPt = &(NFComBuf->ReadDrivesStatus.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_ReadDrivesStatus){
							*(dataPt + txDataIter*NF_DATABYTES_ReadDrivesStatus + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						txDataIter++;						
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadDrivesStatus;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadDrivesStatus;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadDrivesStatus;
					txBuf[txBufIter+1] = 0;	
					txBufIter += 2;
					
				}
			}
			else
		#endif

		// ########	Analog IO
		// ####		Read Inputs
		#ifdef NF_BUFSZ_ReadAnalogInputs
			if(commandArray[commandIter] == NF_COMMAND_ReadAnalogInputs){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf->myAddress){					
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadAnalogInputs) {
						u8TempPt = &(NFComBuf->ReadAnalogInputs.data[combufDataIter]);
						dataBytesIter = 0;
						while(dataBytesIter < NF_DATABYTES_ReadAnalogInputs){
							*(dataPt + txDataIter*NF_DATABYTES_ReadAnalogInputs + dataBytesIter) = *(u8TempPt + dataBytesIter);
							dataBytesIter++;
						}
						txDataIter++;						
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadAnalogInputs;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadAnalogInputs;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadAnalogInputs;
					txBuf[txBufIter+1] = 0;	
					txBufIter += 2;
					
				}
			}
			else
		#endif
		
			{
				NFComBuf->unknownCommandSend = 1;  
				commandIter++;
				continue;
			}
		commandIter++;
	}
	txBuf[txBufIter] = crcFast(((const uint8_t*)txBuf) + 3,  txBufIter-3);
	txBuf[1] = txBufIter;
	txBuf[2] = ~txBufIter;


	return txBufIter+1;
}

void NF_ComBufReset(NF_STRUCT_ComBuf *NFComBuf){
	uint16_t combufDataIter;
	// ########	Device
	// ####		Read Status
	#ifdef NF_BUFSZ_ReadDeviceStatus
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_ReadDrivesStatus; combufDataIter++){
			NFComBuf->ReadDrivesStatus.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->ReadDrivesStatus.data[combufDataIter] = 0;
		}
	#endif
	// ####		Read Vitals
	#ifdef NF_BUFSZ_ReadDeviceVitals
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_ReadDeviceVitals; combufDataIter++){
			NFComBuf->ReadDeviceVitals.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->ReadDeviceVitals.data[combufDataIter] = 0;
		}
	#endif
	
	// ########	Drives
	// ####		Set Mode
	#ifdef NF_BUFSZ_SetDrivesMode
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetDrivesMode; combufDataIter++){
			NFComBuf->SetDrivesMode.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetDrivesMode.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set Speed
	#ifdef NF_BUFSZ_SetDrivesSpeed
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetDrivesSpeed; combufDataIter++){
			NFComBuf->SetDrivesSpeed.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetDrivesSpeed.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set Current
	#ifdef NF_BUFSZ_SetDrivesCurrent
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetDrivesCurrent; combufDataIter++){
			NFComBuf->SetDrivesCurrent.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetDrivesCurrent.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set Position
	#ifdef NF_BUFSZ_SetDrivesPosition
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetDrivesPosition; combufDataIter++){
			NFComBuf->SetDrivesPosition.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetDrivesPosition.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set PWM
	#ifdef NF_BUFSZ_SetDrivesPWM
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetDrivesPWM; combufDataIter++){
			NFComBuf->SetDrivesPWM.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetDrivesPWM.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set Max Current
	#ifdef NF_BUFSZ_SetDrivesMaxCurrent
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetDrivesMaxCurrent; combufDataIter++){
			NFComBuf->SetDrivesMaxCurrent.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetDrivesMaxCurrent.data[combufDataIter] = 0;
		}
	#endif
	// ####		Read Current
	#ifdef NF_BUFSZ_ReadDrivesCurrent
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_ReadDrivesCurrent; combufDataIter++){
			NFComBuf->ReadDrivesCurrent.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->ReadDrivesCurrent.data[combufDataIter] = 0;
		}
	#endif
	// ####		Read Position
	#ifdef NF_BUFSZ_ReadDrivesPosition
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_ReadDrivesPosition; combufDataIter++){
			NFComBuf->ReadDrivesPosition.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->ReadDrivesPosition.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set Misc
	#ifdef NF_BUFSZ_SetDrivesMisc
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetDrivesMisc; combufDataIter++){
			NFComBuf->SetDrivesMisc.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetDrivesMisc.data[combufDataIter] = 0;
		}
	#endif
	// ####		Read Status
	#ifdef NF_BUFSZ_ReadDrivesStatus
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_ReadDrivesStatus; combufDataIter++){
			NFComBuf->ReadDrivesStatus.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->ReadDrivesStatus.data[combufDataIter] = 0;
		}
	#endif
	// ########	Servos
	// ####		Set Mode
	#ifdef NF_BUFSZ_SetServosMode
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetServosMode; combufDataIter++){
			NFComBuf->SetServosMode.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetServosMode.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set Position
	#ifdef NF_BUFSZ_SetServosPosition
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetServosPosition; combufDataIter++){
			NFComBuf->SetServosPosition.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetServosPosition.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set Speed
	#ifdef NF_BUFSZ_SetServosSpeed
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetServosSpeed; combufDataIter++){
			NFComBuf->SetServosSpeed.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetServosSpeed.data[combufDataIter] = 0;
		}
	#endif
	
	// ########	Digital IO	 
	// ####		Read Inputs
	#ifdef NF_BUFSZ_ReadDigitalInputs
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_ReadDigitalInputs; combufDataIter++){
			NFComBuf->ReadDigitalInputs.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->ReadDigitalInputs.data[combufDataIter] = 0;
		}
	#endif
	// ####		Set Outputs
	#ifdef NF_BUFSZ_SetDigitalOutputs
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_SetDigitalOutputs; combufDataIter++){
			NFComBuf->SetDigitalOutputs.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->SetDigitalOutputs.data[combufDataIter] = 0;
		}
	#endif
	
	// ########	Analog IO
	// ####		Read Inputs
	#ifdef NF_BUFSZ_ReadAnalogInputs
		for(combufDataIter=0; combufDataIter<NF_DATABYTES_ReadAnalogInputs; combufDataIter++){
			NFComBuf->ReadAnalogInputs.addr[combufDataIter] = NF_BroadcastAddress;
			NFComBuf->ReadAnalogInputs.data[combufDataIter] = 0;
		}
	#endif
}

