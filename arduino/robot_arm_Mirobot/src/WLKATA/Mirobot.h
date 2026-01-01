#ifndef _MIROBOT_H
#define _MIROBOT_H

#include <Arduino.h>
#include "base.h"
#include "UARTMaster.h"

// Path mode
#define MOVEP 0	// Point-to-point motion
#define MOVEL 1	// Linear motion
#define MOVEJ 1	// Joint motion
#define JUMP 	2	// Door-shaped motion
// Motion mode
#define ABS 0	// Absolute position
#define INC 1	// Relative position
// Arc direction
#define CW 0	// Clockwise arc
#define CCW 1	// Counterclockwise arc

#define STATEDELAY 0

struct STATUS_MIROBOT{
	int state=-1;           // Motion state 
	float angle[7];         // Joint angles
	float cartesian[6];     // Cartesian coordinates
	int pumpPwm;            // PWM output
};

class Mirobot_UART{
  public:
		Mirobot_UART(UARTMaster *p);
    void init(int addr = -1);  // Communication serial port initialization. Parameter: serial port pointer, address
		void sendMsg(String str, bool askEn=ON);  // Send command
		void receive();                           // Receive command
		String getVersions();                     // Get basic information
		int getState();                           // Read motion state
		String getStateToStr();                   // Read motion state as string
		STATUS_MIROBOT getStatus();               // Read all states
		void waitIdle();                          // Wait until idle state
		
		void homing(int homingMode = -1);         // Homing
		void zero();                              // Return to zero point
		
		void movePose(uint8_t pathMode, bool motionMode, float x, float y, float z, float a, float b, float c); // Robot arm position motion
		void movePoseWithExj(uint8_t pathMode, bool motionMode, float x, float y, float z, float a, float b, float c, float d); // Robot arm position motion with extension axis
		void moveArc(bool pathMode, bool motionMode, float x, float y, float z, float r); // Robot arm arc interpolation motion
		void moveJoints(bool motionMode, float j1, float j2, float j3, float j4, float j5, float j6); // Robot arm joint motion

		void setMotionSpeed(float speed);         // Set motion speed
		void motionSpeedRatio(uint8_t ratio);     // Set motion speed percentage
		//void motionParams();                    // Set motion parameters
		
		void movePause();     // Pause motion
		void moveContinue();  // Continue motion
		void moveStop();      // Stop motion
		
		// Extension axis
		void setExjRatio(float ratio);                // Set extension axis reduction ratio
		void moveExjPulse(bool motionMode, int32_t d);// Extension axis motion - unit: pulses
		void moveExjDist(bool motionMode, float d);   // Extension axis motion - unit: distance
		
		// End tools
    void setEndtGripper(uint8_t num);   // Control electric gripper state
    void setEndtPump(uint8_t num);      // Control air pump state
    void setEndtPwm(uint16_t num);      // Control PWM output pulse width
		
		//void saveParams();               // Save parameters
		
		void runFile(String fileName, bool loop = false);   // Run by file name
		void runFileNum(uint8_t fileNum, bool loop = false);// Run by file number
		
		void reset(); // Device restart
		
  protected:
		STATUS_MIROBOT status;    // All state information
		UARTMaster *pSerial;      // Serial port pointer
		int address;              // Device address
		uint32_t sendTime;        // Data sending time
		float exj_ratio=1;        // Extension axis reduction ratio (pulses)
		String firmwareVer="";    // Firmware version
		String exboxVer="";       // Extension box version
		String versions="";       // Version (not used for now)
		
		bool findNum(String* str, String fStr, uint8_t num, String* retStr); // Find specific string
		bool findState(String* str, char c1, char c2);                       // Find state string
		bool dealstatus(String* str);                                        // Parse returned state
		
};

#endif