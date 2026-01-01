#ifndef _UARTMaster_H
#define _UARTMaster_H

#include <Arduino.h>

#define ReceiveDelay 100
#define OUTTIME 500

class UARTMaster{
  public:
    UARTMaster(HardwareSerial* hwSerial); // Communication serial port, parameter: serial pointer
		
    void begin(unsigned long baudrate);	// Initialize, set baud rate
		void begin(HardwareSerial* hwSerial, unsigned long baudrate);	// Initialize, set port and baud rate
    void beginTransmission(int addr, bool waitAckEN = true);	// Start sending data
		void writeData(String str);	// Send data
    void endTransmission();	// Finish sending
		void send(String str, int addr, bool waitAckEN = true);	// Send data
		void resend();	// Resend data
		uint8_t getAckState();// Acknowledgment state
		String receive();	// Receive command
		void setOutTimeFunction(void (*_func)(String, int), uint32_t time = OUTTIME); // Set timeout callback function
		void setOutTime(uint32_t time); // Set timeout
		void setMonitorFunction(void (*_func)(String, bool)); // Set monitoring callback function
		
  protected:
		HardwareSerial* pSerial;
		unsigned long baudrate;
		uint8_t ackState = 0; // Acknowledgment state
		String sendStr; // Sent string
		String receiveStr; // Received string
		int address; // Device address
		char sendBuf[128]; // Send buffer
		char receiveBuf[128]; // Receive buffer
		uint8_t sendLen; // Send string length
		uint8_t receiveLen; // Received string length
		uint32_t sendTime; // Data send timestamp
		bool noAskFunctionFlag = false;
		void (*outTimeFunc)(String, int); // Timeout callback function
		uint32_t ackMaxTime = OUTTIME; // Maximum acknowledgment time (ms)
		bool monitorFunctionFlag = false;
		void (*monitorFunc)(String, bool); // Monitoring callback function
};

#endif