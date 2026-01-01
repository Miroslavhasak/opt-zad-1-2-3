#include "UARTMaster.h"


UARTMaster::UARTMaster(HardwareSerial* hwSerial){
	pSerial = hwSerial;
}

/*
  @Description  Set communication-related parameters for the device
  @Parameter    *p     Serial port
  @Parameter    addr   Device address, range: 1–247, 0 = broadcast address
                       If this serial port uses UART (not RS485), 
                       the address can be left unset or set to -1
  @Return       None
*/
void UARTMaster::begin(unsigned long baud){
	baudrate = baud;
	pSerial->begin(baudrate);
	pSerial->setTimeout(ReceiveDelay);
	//pSerial->println("Debug,test");
}

void UARTMaster::begin(HardwareSerial* hwSerial, unsigned long baud){
	pSerial = hwSerial;
	begin(baud);
}

void UARTMaster::beginTransmission(int addr, bool waitAckEN){
	while(pSerial->available()) pSerial->read(); // Clear buffer
	address = addr;
	if(addr != -1){
		sendStr = "@" + String(addr); // Generate header string
		pSerial->print(sendStr); // Send
	}
	else sendStr = "";
	
	// Acknowledgment state
	if(addr != 0 && waitAckEN) ackState = 1;
	else ackState = 0;
}

void UARTMaster::writeData(String str){
	pSerial->print(str); // Send
	sendStr += str;

}

void UARTMaster::endTransmission(){
	sendTime = millis(); // Record send timestamp
	if(monitorFunctionFlag) (*monitorFunc)(sendStr, 1); // Send string to monitor callback
}

void UARTMaster::send(String str, int addr, bool waitAckEN){
	beginTransmission(addr, waitAckEN);
	writeData(str);
	endTransmission();
}

uint8_t UARTMaster::getAckState(){
	pSerial->flush(); // Wait for sending to finish
	if(ackState == 1){ // Waiting for acknowledgment
		while(1){
			if(pSerial->available()){
				receiveStr = pSerial->readStringUntil('\n'); // Read
				if(receiveStr.endsWith("\r")){ // Complete return with correct format
					receiveStr += '\n';
					ackState = 2;
					if(monitorFunctionFlag) (*monitorFunc)(receiveStr, 0); // Send received string to monitor callback
				}
				else{ // Incomplete return format
					receiveStr = "";
					ackState = 0;
				}
				break;
			}
			else if(millis()-sendTime > OUTTIME){ // No acknowledgment or acknowledgment timeout
				ackState = 0;
				if(noAskFunctionFlag) (*outTimeFunc)(sendStr, address);
				break;
			}
		}
	}
	else if(ackState == 3){
		receiveStr = pSerial->readStringUntil('\n'); // Read
		if(receiveStr.endsWith("\r")){
			receiveStr += '\n';
			ackState = 2;
			if(monitorFunctionFlag) (*monitorFunc)(receiveStr, 0); // Send received string to monitor callback
		}
		else{
			receiveStr = "";
			ackState = 0;
		}
	}
	return ackState;
}

String UARTMaster::receive(){
	String ret = "";
	if(getAckState() == 2){
		ret = receiveStr;
		if(getAckState()) ackState = 3;
		else ackState = 0;
	}
	return ret;
}

void UARTMaster::setOutTimeFunction(void (*_func)(String, int), uint32_t time){
	noAskFunctionFlag = true;
	outTimeFunc = _func;
	ackMaxTime = time;
}

void UARTMaster::setOutTime(uint32_t time){
	ackMaxTime = time;
}

void UARTMaster::setMonitorFunction(void (*_func)(String, bool)){
	monitorFunctionFlag = true;
	monitorFunc = _func;
}