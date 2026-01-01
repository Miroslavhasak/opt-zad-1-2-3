#include "WSerial.h"

#pragma once

#include "robot.h"
#include "src\WLKATA\Mirobot.h"


class MirobotController: public Robot::JointActuators {
public:

  MirobotController(HardwareSerial* SerialHandle, const std::vector<actuator_parameters>& parameters_)
    : Robot::JointActuators(), parameters(parameters_), robot_UART(SerialHandle), mirobot(&robot_UART) {}
  void init() {
    mirobot.init(-1);
    mirobot.setMotionSpeed(2000);
  }
  bool writeAngles(const float* angles, std::size_t n) {

    //mirobot.waitIdle();
    float degrees[6];
    for (int i = 0; i < n; i++) {
      if (angles[i] > parameters[i].maxAngle || angles[i] < parameters[i].minAngle) return false;
      degrees[i] = (angles[i] - parameters[i].offset) * parameters[i].gain * 180.0f / PI;
    }
    mirobot.moveJoints(ABS, degrees[0], degrees[1], degrees[2], degrees[3], degrees[4], degrees[5]);
    return true;
  }

  bool readAngles(float* angles, std::size_t n) {
    //STATUS_MIROBOT status = mirobot.getStatus();
    return true;
  }

private:
  std::vector<actuator_parameters> parameters;
  UARTMaster robot_UART;
  Mirobot_UART mirobot;
};
