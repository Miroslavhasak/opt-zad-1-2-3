#include "base.h"

const String stateStr[7]={"Offline", "Idle", "Alarm", "Home", "Run", "Hold", "Busy"};

String stateToStr(int num){
	return stateStr[num];
}

