#include "Mirobot.h"

Mirobot_UART::Mirobot_UART(UARTMaster *p){
	pSerial = p;
}

/*
  @Description	Set communication parameters of the device
  @Parameter	*p		Serial port
  @Parameter	addr	Device address, range: 1-247, 0 is broadcast address
							If this port uses UART instead of RS485, address can be omitted or set to -1
  @Return None
*/
void Mirobot_UART::init(int addr){
	address = addr;
}

/*
  @Description	Send command, optionally wait for response
  @Parameter	str		String to send, no need to include address (added automatically)
  @Parameter	askEn	Response wait enable (0: don't wait, 1: wait)
  @Return None
*/
void Mirobot_UART::sendMsg(String str, bool askEn){
	pSerial->send(str, address, askEn);
	sendTime = millis();
}

/*
  @Description	Receive command, optionally wait for response
  @Parameter	None
*/
void Mirobot_UART::receive(){
	while(pSerial->getAckState()){
		
		String str = pSerial->receive();

		if(str.startsWith("State")) findState(&str, ',',',');
		else if(str.startsWith("<")) dealstatus(&str);
		else if(str.startsWith("Mirobot") || str.startsWith("Dark3") || str.startsWith("E4")){
			uint8_t size=str.length();
			firmwareVer = str.substring(0,size-2);
		}
		else if(str.startsWith("EXbox")){
			uint8_t size=str.length();
			exboxVer = str.substring(0,size-2);
		}
		else if(str.endsWith("ok\r\n"));
		else;
	}

}

/*
  @Description	Get basic information, version numbers
  @Parameter	None
  @Return String
*/
String Mirobot_UART::getVersions(){
	firmwareVer = "";
	exboxVer = "";
	sendMsg("$v\r\n", 1);
	receive();
	if(firmwareVer == ""){
		receive();
	}
	return firmwareVer + "\r\n" + exboxVer + "\r\n";
}

/*
  @Description	Read motion state
  @Parameter	None
  @Return int
*/
int Mirobot_UART::getState(){
	while(millis()-sendTime < STATEDELAY); // Reserve time for sending
	sendMsg("O103\r\n", 1);
	receive();
	return status.state;
}

/*
  @Description	Read motion state as string
  @Parameter	None
  @Return String
*/
String Mirobot_UART::getStateToStr(){
	while(millis()-sendTime < STATEDELAY); // Reserve time for sending
	int num = getState();
	if(num==-1) return "Error";
	return stateToStr(num);
}

/*
  @Description	Read all status
  @Parameter	None
  @Return STATUS_MIROBOT
*/
STATUS_MIROBOT Mirobot_UART::getStatus(){
	while(millis()-sendTime < STATEDELAY); // Reserve time for sending
	sendMsg("?\r\n", 1);
	receive();
	return status;
}

/*
  @Description	Wait until idle
  @Parameter	None
  @Return None
*/
void Mirobot_UART::waitIdle(){
	while(getState() != Idle); // Wait for idle
}

/*
  @Description	Homing
  @Parameter	mode	0/$H: homing device and extension axes together
							1/$H1: homing axis 1
							2/$H2: homing axis 2
							3/$H3: homing axis 3
							4/$H4: homing axis 4
							5/$H5: homing axis 5
							6/$H6: homing axis 6 (only specific versions)
							7/$H7: homing extension axis (requires sensor)
							8/$H: homing all device axes together
							9/$HH: homing all device axes separately
							10/$HE: homing device axes to minimum pose
  @Return None
*/
void Mirobot_UART::homing(int mode){
  switch(mode){
		case 0: sendMsg("O105=0\r\n", 1); break; //$H0
    case 1: sendMsg("O105=1\r\n", 1); break; //$H1
    case 2: sendMsg("O105=2\r\n", 1); break; //$H2
    case 3: sendMsg("O105=3\r\n", 1); break; //$H3
    case 4: sendMsg("O105=4\r\n", 1); break; //$H4
    case 5: sendMsg("O105=5\r\n", 1); break; //$H5
    case 6: sendMsg("O105=6\r\n", 1); break; //$H6
    case 7: sendMsg("O105=7\r\n", 1); break; //$H7
		case 8: sendMsg("O105\r\n", 1); break; //$H
		case 9: sendMsg("O105=9\r\n", 1); break; //$HH
		case 10: sendMsg("O105=10\r\n", 1); break; //$HE
    default: sendMsg("O105\r\n", 1); break; //$H
  }
	receive();
}

/*
  @Description	Move to zero position
  @Parameter	None
  @Return None
*/
void Mirobot_UART::zero(){    // Initial position of device
  sendMsg("M21 G90 G00 X0 Y0 Z0 A0 B0 C00\r\n", 1); // Joint mode to zero
	receive();
}

/*
  @Description	Move robotic arm
  @Parameter	pathMode	Path mode: MOVEP point-to-point, MOVEL linear, JUMP gate
	@Parameter	motionMode	Move mode: ABS absolute, INC relative
	@Parameter	x		x coordinate
	@Parameter	y		y coordinate
	@Parameter	z		z coordinate
	@Parameter	a		a coordinate
	@Parameter	b		b coordinate
	@Parameter	c		c coordinate
  @Return None
*/
void Mirobot_UART::movePose(uint8_t pathMode, bool motionMode, float x, float y, float z, float a, float b, float c){
	String str = "";
	str += "M20";
	switch(motionMode){
		case ABS: str += "G90"; break;
		case INC: str += "G91"; break;
	}
	switch(pathMode){
		case MOVEP: str += "G00"; break;
		case MOVEL: str += "G01"; break;
		case JUMP : str += "G05"; break;
	}
	str += "X"; str += x;
	str += "Y"; str += y;
	str += "Z"; str += z;
	str += "A"; str += a;
	str += "B"; str += b;
	str += "C"; str += c;
	str += "\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	Move robotic arm with extension axis
  @Parameter	pathMode	Path mode: MOVEP point-to-point, MOVEL linear, JUMP gate
	@Parameter	motionMode	Move mode: ABS absolute, INC relative
	@Parameter	x		x coordinate
	@Parameter	y		y coordinate
	@Parameter	z		z coordinate
	@Parameter	a		a coordinate
	@Parameter	b		b coordinate
	@Parameter	c		c coordinate
	@Parameter	d		d coordinate (extension axis)
  @Return None
*/
void Mirobot_UART::movePoseWithExj(uint8_t pathMode, bool motionMode, float x, float y, float z, float a, float b, float c, float d){
	String str = "";
	str += "M20";
	switch(motionMode){
		case ABS: str += "G90"; break;
		case INC: str += "G91"; break;
	}
	switch(pathMode){
		case MOVEP: str += "G00"; break;
		case MOVEL: str += "G01"; break;
		case JUMP : str += "G05"; break;
	}
	str += "X"; str += x;
	str += "Y"; str += y;
	str += "Z"; str += z;
	str += "A"; str += a;
	str += "B"; str += b;
	str += "C"; str += c;
	str += "D"; str += d;
	str += "\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	Robotic arm arc interpolation
  @Parameter	pathMode	Path mode: CW clockwise, CCW counterclockwise
	@Parameter	motionMode	Move mode: ABS absolute, INC relative
	@Parameter	x		x coordinate
	@Parameter	y		y coordinate
	@Parameter	z		z coordinate
	@Parameter	r		arc radius
  @Return None
*/
void Mirobot_UART::moveArc(bool pathMode, bool motionMode, float x, float y, float z, float r){
	String str = "";
	str += "M20";
	switch(motionMode){
		case ABS: str += "G90"; break;
		case INC: str += "G91"; break;
	}
	switch(pathMode){
		case CW : str += "G02"; break;
		case CCW: str += "G03"; break;
	}
	str += "X"; str += x;
	str += "Y"; str += y;
	str += "Z"; str += z;
	str += "R"; str += r;
	str += "\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	Robotic arm joint motion
  @Parameter	motionMode	Move mode: ABS absolute, INC relative
	@Parameter	j1		Joint 1 angle
	@Parameter	j2		Joint 2 angle
	@Parameter	j3		Joint 3 angle
	@Parameter	j4		Joint 4 angle
	@Parameter	j5		Joint 5 angle
	@Parameter	j6		Joint 6 angle
  @Return None
*/
void Mirobot_UART::moveJoints(bool motionMode, float j1, float j2, float j3, float j4, float j5, float j6){
	String str = "";
	str += "M21";
	switch(motionMode){
		case ABS: str += "G90"; break;
		case INC: str += "G91"; break;
	}
	//str += "G01";
	str += "G00";
	str += "X"; str += j1;
	str += "Y"; str += j2;
	str += "Z"; str += j3;
	str += "A"; str += j4;
	str += "B"; str += j5;
	str += "C"; str += j6;
	str += "\r\n";
	sendMsg(str, 1);
	receive();

}

/*
  @Description	Set motion speed
	@Parameter	speed		Speed value
  @Return None
*/
void Mirobot_UART::setMotionSpeed(float speed){
	String str = "";
	str += "F";
	str += speed;
	str += "\r\n";
  sendMsg(str, 1);
	receive();
}

/*
  @Description	Set motion speed ratio
	@Parameter	ratio		Speed percentage (1-100)
  @Return None
*/
void Mirobot_UART::motionSpeedRatio(uint8_t ratio){
	String str = "";
	str += "H";
	str += ratio;
	str += "\r\n";
  sendMsg(str, 1);
	receive();
}

/*
  @Description	Pause motion
	@Parameter	None
  @Return None
*/
void Mirobot_UART::movePause(){
	sendMsg("!\r\n", 0);
	receive();
}

/*
  @Description	Continue motion
	@Parameter	None
  @Return None
*/
void Mirobot_UART::moveContinue(){
	sendMsg("~\r\n", 0);
	receive();
}

/*
  @Description	Stop motion
	@Parameter	None
  @Return None
*/
void Mirobot_UART::moveStop(){
	sendMsg("%\r\n", 0);
	receive();
}

/*
  @Description	Set extension axis pulse ratio
	@Parameter	ratio		Pulses per unit
  @Return None
*/
void Mirobot_UART::setExjRatio(float ratio){
	exj_ratio = ratio;
}

/*
  @Description	Move extension axis by pulses
  @Parameter	motionMode	Move mode: ABS absolute, INC relative
	@Parameter	d		Extension axis value
  @Return None
*/
void Mirobot_UART::moveExjPulse(bool motionMode, int32_t d){
	String str = "";
	str += "M21";
	switch(motionMode){
		case ABS: str += "G90"; break;
		case INC: str += "G91"; break;
	}
	str += "G00 D";
	str += d;
	str += "\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	Move extension axis by distance
  @Parameter	motionMode	Move mode: ABS absolute, INC relative
	@Parameter	d		Extension axis value
  @Return None
*/
void Mirobot_UART::moveExjDist(bool motionMode, float d){
	String str = "";
	str += "M21";
	switch(motionMode){
		case ABS: str += "G90"; break;
		case INC: str += "G91"; break;
	}
	str += "G00 D";
	str += d*exj_ratio;
	str += "\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	Control electric gripper
  @Parameter	num		Gripper state (0/OFF: close, 1/OPEN: open, 2/CLOSE: close)
  @Return None
*/	
void Mirobot_UART::setEndtGripper(uint8_t num){
	String str = "";
	if(num == OFF) str = "M3 S0\r\n";
	else if(num == OPEN) str = "M3 S40\r\n";
	else if(num == CLOSE) str = "M3 S60\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	Control vacuum pump
  @Parameter	num		PWM duty (0/OFF: off, 1/IN: positive, 2/OUT: negative)
  @Return None
*/	
void Mirobot_UART::setEndtPump(uint8_t num){
	String str = "";
	if(num == OFF) str = "M3 S0\r\n";
	else if(num == OUT) str = "M3 S500\r\n";
	else if(num == IN) str = "M3 S1000\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	PWM output, yellow interface of controller
  @Parameter	num		PWM duty (0-1000)
  @Return None
*/	
void Mirobot_UART::setEndtPwm(uint16_t num){
	sendMsg("M3 S"+String(num)+"\r\n", 1);
	receive();
}

/*
  @Description	Run file by name
  @Parameter	fileName	File name
	@Parameter	loop	Loop flag (false: no loop, true: loop)
  @Return None
*/	
void Mirobot_UART::runFile(String fileName, bool loop){
	String str = "";
	if(loop) str += "O112=";
	else str += "O111=";
	str += fileName;
	str += "\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	Run file by number
  @Parameter	fileNum	File number (first two digits of file name)
	@Parameter	loop	Loop flag (false: no loop, true: loop)
  @Return None
*/
void Mirobot_UART::runFileNum(uint8_t fileNum, bool loop){
	String str = "";
	if(loop) str += "O116=";
	else str += "O116=";
	str += fileNum;
	str += "\r\n";
	sendMsg(str, 1);
	receive();
}

/*
  @Description	Reset device
  @Parameter	None
  @Return None
*/
void Mirobot_UART::reset(){
	sendMsg("O100\r\n", 1);
	receive();
}

/*
  @Description	Find data after specified string
  @Parameter	str			String to search
	@Parameter	fStr		String to find
	@Parameter	num			Which occurrence after string (min 1)
	@Parameter	retStr		Result
  @Return bool 	Success flag (0: fail, 1: success)
*/
bool Mirobot_UART::findNum(String* str, String fStr, uint8_t num, String* retStr){
	uint8_t startAddr, endAddr;
	startAddr = (*str).indexOf(fStr);
	if(startAddr == -1) return 0;
	endAddr = (*str).indexOf(':', startAddr);
	if(endAddr == -1) return 0;
	for(uint8_t i=0; i<num; i++){
		startAddr = endAddr+1;
		endAddr = (*str).indexOf(',', startAddr);
		if(endAddr == -1) return 0;
	}
	*retStr = (*str).substring(startAddr, endAddr);
	return 1;
}

/*
  @Description	Find motion state in string
  @Parameter	str		String containing state
	@Parameter	c1		Start character
	@Parameter	c2		End character
  @Return bool 	Success flag (0: fail, 1: success)
*/
bool Mirobot_UART::findState(String* str, char c1, char c2){
	uint8_t startAddr, endAddr;
	status.state = -1;
	startAddr = (*str).indexOf(c1);
	if(startAddr == -1) return 0;
	startAddr++;
	endAddr = (*str).indexOf(c2, startAddr);
	if(endAddr == -1) return 0;
	String s = (*str).substring(startAddr, endAddr);
	for(uint8_t i=0; i<7; i++){
		if(s == stateToStr(i)){
			status.state = i;
			return 1;
		}
	}
	return 0;
}

/*
  @Description	Parse returned status
  @Parameter	str		Status string
  @Return bool 	Success flag (0: fail, 1: success)
*/
bool Mirobot_UART::dealstatus(String* str){
	String s;
	uint8_t i;
	// Get state
	if(findState(str, '<',',')==0) return 0;
	// Get angles
	for(i=0; i<7; i++){
		if(findNum(str, "Angle", i+1, &s)==0) return 0;
		status.angle[(i+3)%7] = s.toFloat();
	}
	// Get coord
	for(i=0; i<6; i++){
		if(findNum(str, "Cartesian", i+1, &s)==0) return 0;
		status.cartesian[i] = s.toFloat();
	}
	// Get pwm
	if(findNum(str, "Pump", 1, &s)==0) return 0;
	status.pumpPwm = s.toInt();
	return 1;	
}