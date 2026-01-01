

#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include "Eigen/Dense"

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "src\LiquidMenu\LiquidCrystal_I2C.h"
#include "src\LiquidMenu\LiquidMenu.h"
#include "src\Button.h"
#include "src\Joystick.h"
#include "src\Keypad\Keypad.h"
#include "src\data_stream\data_stream.h"
#include "command_parser.h"


#include "ServoActuator.h"
#include "robot.h"
#include "interpolation.h"


TaskHandle_t HandleTaskControl;
TaskHandle_t HandleTaskUI;
TaskHandle_t HandleTaskCOM;
TaskHandle_t HandleTaskCMD;
TaskHandle_t HandleTaskSensor;

SemaphoreHandle_t UISemaphore;
SemaphoreHandle_t ControlSemaphore;
SemaphoreHandle_t SensorSemaphore;
SemaphoreHandle_t collisionSemaphore;
SemaphoreHandle_t nocollisionSemaphore;
SemaphoreHandle_t RobotIdleSemaphore;

TimerHandle_t UITimer;
TimerHandle_t ControlTimer;
TimerHandle_t SensorTimer;



QueueHandle_t UI_effector_position_orientation_Queue;
QueueHandle_t sensor_effector_position_orientation_Queue;
QueueHandle_t sensor_point_Queue;
QueueHandle_t desired_effector_position_orientation_Queue;
QueueHandle_t thetas_Queue;
QueueHandle_t CMD_Queue;
QueueHandle_t data_stream_thetas_Queue;
QueueHandle_t data_stream_effector_position_orientation_Queue;
QueueHandle_t IK_succ_Queue;

constexpr std::size_t DOF = 6;

typedef enum {
  control_thetas = 0,
  control_position = 1,
  control_orientation = 2,
  control_CMD = 3,
} control_mode_t;

typedef enum {
  XY = 0,
  XZ = 1,
  YZ = 2,
} position_pair_t;

typedef enum {
  rp = 0,
  ry = 1,
  py = 2,
} orientation_pair_t;

typedef struct
{
  control_mode_t mode;
  orientation_pair_t orientation_control;
  position_pair_t position_control;
  int joint_number;
} control_settings;

control_settings settings = { control_position, rp, XY, 0 };

void setup(void) {

  delay(100);
  Serial.begin(250000);
  Wire.begin();

  ControlSemaphore = xSemaphoreCreateBinary();
  UISemaphore = xSemaphoreCreateBinary();
  SensorSemaphore = xSemaphoreCreateBinary();
  collisionSemaphore = xSemaphoreCreateBinary();
  nocollisionSemaphore = xSemaphoreCreateBinary();
  RobotIdleSemaphore = xSemaphoreCreateBinary();

  UI_effector_position_orientation_Queue = xQueueCreate(1, sizeof(positionOrientation));
  sensor_effector_position_orientation_Queue = xQueueCreate(1, sizeof(positionOrientation));
  desired_effector_position_orientation_Queue = xQueueCreate(1, sizeof(positionOrientation));
  IK_succ_Queue = xQueueCreate(1, sizeof(bool));
  thetas_Queue = xQueueCreate(1, sizeof(Matrix<DOF, 1>));
  CMD_Queue = xQueueCreate(30, sizeof(PoseCommand));
  data_stream_thetas_Queue = xQueueCreate(1, sizeof(Matrix<DOF, 1>));
  data_stream_effector_position_orientation_Queue = xQueueCreate(1, sizeof(positionOrientation));
  sensor_point_Queue = xQueueCreate(20, sizeof(Matrix<3, 1>));

  xTaskCreate(TaskControl,
              "Control",
              5000,
              NULL,
              tskIDLE_PRIORITY + 4,
              &HandleTaskControl);

  xTaskCreate(TaskUI,
              "UI",
              1500,
              NULL,
              tskIDLE_PRIORITY + 2,
              &HandleTaskUI);

  
 
  xTaskCreate(TaskCom,
              "Com",
              500,
              NULL,
              tskIDLE_PRIORITY + 1,
              &HandleTaskCOM);


  xTaskCreate(TaskCMD,
              "CMD",
              2000,
              NULL,
              tskIDLE_PRIORITY + 1,
              &HandleTaskCMD);


xTaskCreate(TaskSensor,
              "Sensor",
              1000,
              NULL,
              tskIDLE_PRIORITY + 3,
              &HandleTaskSensor);

vTaskStartScheduler();
}

void loop(void) {
}


void TaskUI(void* pvParameters) {

  const int update_rate = 50;

  LiquidCrystal_I2C lcd(0x27, 16, 2);
  lcd.init();
  lcd.backlight();

  const char keys[4][4] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '8', '9', 'C' },
    { '.', '0', '-', 'D' }
  };
  byte rowPins[4] = { 4, 5, 6, 7 };
  byte colPins[4] = { 8, 9, 10, 11 };

  Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, 4, 4);
  Button button_variable(12, true);
  Joystick joystick(A1, A0);


  positionOrientation effector_position_orientation;
  positionOrientation desired_effector_position_orientation;


  desired_effector_position_orientation.X = 0.0;
  desired_effector_position_orientation.Y = 0.1;
  desired_effector_position_orientation.Z = 0.07;
  desired_effector_position_orientation.r = -PI;
  desired_effector_position_orientation.p = 0;
  desired_effector_position_orientation.y = 0;

  Matrix<DOF, 1> thetas = Matrix<DOF, 1>::Zero();

  bool IK_succ = true;

  char mode_string[10] = "pos";
  char aux_string[10] = "XY";

  LiquidLine lineMode(8, 1, mode_string);
  LiquidLine lineAux(12, 1, aux_string);

  LiquidLine lineX(0, 0, "X=", effector_position_orientation.X);
  lineX.set_decimalPlaces(2);
  LiquidLine lineY(8, 0, "Y=", effector_position_orientation.Y);
  lineY.set_decimalPlaces(2);
  LiquidLine lineZ(0, 1, "Z=", effector_position_orientation.Z);
  lineZ.set_decimalPlaces(2);
  LiquidScreen screenXYZ(lineX, lineY, lineZ);
  screenXYZ.add_line(lineMode);
  screenXYZ.add_line(lineAux);


  LiquidLine liner(0, 0, "r=", effector_position_orientation.r);
  liner.set_decimalPlaces(2);
  LiquidLine linep(8, 0, "p=", effector_position_orientation.p);
  linep.set_decimalPlaces(2);
  LiquidLine liney(0, 1, "y=", effector_position_orientation.y);
  liney.set_decimalPlaces(2);
  LiquidScreen screenRPY(liner, linep, liney);
  screenRPY.add_line(lineMode);
  screenRPY.add_line(lineAux);

  LiquidLine linet1(0, 0, "t1=", thetas(0));
  linet1.set_decimalPlaces(2);
  LiquidLine linet2(8, 0, "t2=", thetas(1));
  linet2.set_decimalPlaces(2);
  LiquidLine linet3(0, 1, "t3=", thetas(2));
  linet3.set_decimalPlaces(2);
  LiquidLine linet4(0, 0, "t4=", thetas(3));
  linet4.set_decimalPlaces(2);
  LiquidLine linet5(8, 0, "t5=", thetas(4));
  linet5.set_decimalPlaces(2);
  LiquidLine linet6(0, 1, "t6=", thetas(5));
  linet6.set_decimalPlaces(2);
  LiquidScreen screenThetas1(linet1, linet2, linet3);
  LiquidScreen screenThetas2(linet4, linet5, linet6);
  screenThetas1.add_line(lineMode);
  screenThetas1.add_line(lineAux);
  screenThetas2.add_line(lineMode);
  screenThetas2.add_line(lineAux);

  LiquidMenu menu(lcd);
  menu.init();
  menu.add_screen(screenXYZ);
  menu.add_screen(screenRPY);
  menu.add_screen(screenThetas1);
  menu.add_screen(screenThetas2);

  UITimer = xTimerCreate("TimerUI",
                         pdMS_TO_TICKS(update_rate),
                         pdTRUE,
                         (void*)0,
                         vTimerUICallback);

  xTimerStart(UITimer, 0);

const float default_rate = 1;
float rate = default_rate;

bool mode_change = false;
bool submode_change = false;

  while (1) {

    xSemaphoreTake(UISemaphore, portMAX_DELAY);
    xQueueReceive(UI_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), 0);

    joystick.read();

    char key = keypad.getKey();

    if (key) {

      if (key == 'A') {
        menu.next_screen();
      }
      if (key == 'B') {
        menu.previous_screen();
      }

      if (key == 'C') {
        menu.switch_focus();
      }

      if (key == 'D') {
        settings.mode = static_cast<control_mode_t>((settings.mode + 1) % 4);
        mode_change = true;
        submode_change = true;
        }
      }
      if (mode_change)
      { 
        mode_change = false;
        if (settings.mode == control_thetas)
          strcpy(mode_string, "ang ");
        else if (settings.mode == control_position)
          strcpy(mode_string, "pos ");
        else if (settings.mode == control_orientation)
          strcpy(mode_string, "ori ");
        else if (settings.mode == control_CMD) {
          strcpy(mode_string, "cmd ");
      }
      menu.softUpdate();  
    }

    if (button_variable.check() == LOW) {

      if (settings.mode == control_thetas) {
        settings.joint_number = ((settings.joint_number + !mode_change) % DOF);
      } else if (settings.mode == control_position) {
        settings.position_control = static_cast<position_pair_t>((settings.position_control + !mode_change) % 3);

      } else if (settings.mode == control_orientation) {
        settings.orientation_control = static_cast<orientation_pair_t>((settings.orientation_control + !mode_change) % 3);
      }

      submode_change = true;
    
  }


      if (submode_change) {
        submode_change=false;

      if (settings.mode == control_thetas) {
        snprintf(aux_string, sizeof(aux_string), "%d  ", settings.joint_number + 1);
      } else if (settings.mode == control_position) {
    
        if (settings.position_control == XY) {
          strcpy(aux_string, "XY ");
        } else if (settings.position_control == XZ) {
          strcpy(aux_string, "XZ ");
        } else if (settings.position_control == YZ) {
          strcpy(aux_string, "YZ ");
        }

      } else if (settings.mode == control_orientation) {

        if (settings.orientation_control == rp) {
          strcpy(aux_string, "rp ");
        } else if (settings.orientation_control == ry) {
          strcpy(aux_string, "ry ");
        } else if (settings.orientation_control == py) {
          strcpy(aux_string, "py ");
        }
      }
      else strcpy(aux_string, "   ");

      menu.softUpdate();
    
  }


    if (settings.mode == control_thetas) {

      thetas(settings.joint_number) += 0.05 * joystick.getX();
      xQueueSend(thetas_Queue, &thetas, 0);
      desired_effector_position_orientation = effector_position_orientation;
    }


    else if (settings.mode == control_position) {

      if (xSemaphoreTake(nocollisionSemaphore, 0) == pdTRUE)
      {
        rate = default_rate;
      }

      if (xSemaphoreTake(collisionSemaphore, 0) == pdTRUE) {
        strcpy(aux_string, "col ");
        rate = default_rate * 0.1f;
      }


      if (xSemaphoreTake(RobotIdleSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (settings.position_control == XY) {
          desired_effector_position_orientation.X += 0.003 * rate * joystick.getX();
          desired_effector_position_orientation.Y += 0.003 * rate * -joystick.getY();
        } else if (settings.position_control == XZ) {
          desired_effector_position_orientation.X += 0.003 * rate * joystick.getX();
          desired_effector_position_orientation.Z += 0.003 * rate * -joystick.getY();
        } else if (settings.position_control == YZ) {
          desired_effector_position_orientation.Y += 0.003 * rate * -joystick.getY();
          desired_effector_position_orientation.Z += 0.003 * rate * -joystick.getX();
        }
      }
      xQueueReceive(thetas_Queue, &thetas, 0);
    } else if (settings.mode == control_orientation) {

      if (xSemaphoreTake(RobotIdleSemaphore, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (settings.orientation_control == rp) {
          desired_effector_position_orientation.r += 0.025 * rate * joystick.getX();
          desired_effector_position_orientation.p += 0.025 * rate * joystick.getY();
        } else if (settings.orientation_control == ry) {
          desired_effector_position_orientation.r += 0.025 * rate * joystick.getX();
          desired_effector_position_orientation.y += 0.025 * rate * joystick.getY();
        } else if (settings.orientation_control == py) {
          desired_effector_position_orientation.p += 0.025 * rate * joystick.getX();
          desired_effector_position_orientation.y += 0.025 * rate * joystick.getY();
        }
      }
      xQueueReceive(thetas_Queue, &thetas, 0);
    } else if (settings.mode == control_CMD) {
      xQueueReceive(thetas_Queue, &thetas, 0);
    }

    if (settings.mode == control_position || settings.mode == control_orientation || settings.mode == control_CMD) {
      xQueueReceive(IK_succ_Queue, &IK_succ, 0);
      if (!IK_succ) {
        desired_effector_position_orientation = effector_position_orientation;
        strcpy(aux_string, "IKf ");
      }
      xQueueSend(desired_effector_position_orientation_Queue, reinterpret_cast<void*>(&desired_effector_position_orientation), 0);
    }
    menu.softUpdate();
  }
}

void vTimerUICallback(TimerHandle_t xTimer) {
  xSemaphoreGive(UISemaphore);
}


void TaskControl(void* pvParameters) {

const float Ts = 0.025;


  TwoWire Wire2(PF0, PF1);
  Wire2.begin();
  Wire2.setClock(400000);

  std::vector<actuator_parameters> servo_parameters;
  servo_parameters.resize(6);
  servo_parameters[0] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0 };
  servo_parameters[1] = { -5.0 / 10.0 * PI, 5.2 / 10.0 * PI, 2.0 / 3.0, 0.015 };
  servo_parameters[2] = { -8.2 / 10.0 * PI, 5.0 / 10.0 * PI, -2.0 / 3.0, 0.12 };
  servo_parameters[3] = { -7.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, -0.2 };
  servo_parameters[4] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0.0 };
  servo_parameters[5] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, -2.0 / 3.0, 0.0 };

  RobotServoController servo_controller(Wire2, servo_parameters);
  servo_controller.init();

  Matrix<3, 1> T[DOF];
  /*
  T[0] << TODO
  T[1] << TODO
  T[2] << TODO
  T[3] << TODO
  T[4] << TODO
  T[5] << TODO
*/

  RotationZ rotZ_obj;
  RotationX rotX_obj;
  RotationY rotY_obj;
  const RotationMatrix* Rr[DOF] = { &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj };
  Robot::RobotKinematicsRotPtrs<DOF> robot_kinematics(Rr, T);

  Robot::RobotArm<DOF> robot(&robot_kinematics, &servo_controller);

  positionOrientation effector_position_orientation;
  positionOrientation desired_effector_position_orientation;
  PoseInterpolator interpolator;

  xQueueReceive(desired_effector_position_orientation_Queue, reinterpret_cast<void*>(&desired_effector_position_orientation), portMAX_DELAY);

  ControlTimer = xTimerCreate("TimerCon",
                              pdMS_TO_TICKS(Ts * 1e3),
                              pdTRUE,
                              (void*)0,
                              vTimerControlCallback);

  xTimerStart(ControlTimer, 0);

  Matrix<DOF, 1> thetas = Matrix<DOF, 1>::Zero();

  thetas << 0, 0.62, -1.85, -1.9, 0, 0;

  robot.setJointAngles(thetas);

  while (1) {

    xSemaphoreTake(ControlSemaphore, portMAX_DELAY);

    auto [effector_position, R] = robot.getCurrentEffectorPositionOrientation();
    effector_position_orientation = positionOrientation(effector_position, R);


    if (settings.mode == control_thetas) {
      xQueueReceive(thetas_Queue, &thetas, 0);
      robot.setJointAngles(thetas);
    } else if (settings.mode == control_position || settings.mode == control_orientation) {
      xQueueReceive(desired_effector_position_orientation_Queue, reinterpret_cast<void*>(&desired_effector_position_orientation), 0);

      auto Rref = desired_effector_position_orientation.toRotationMatrix();
      auto Re = R * (Rref.transpose());
      double ori_error = (3.0 - Re.trace()) / 4.0;
      double pos_error = (effector_position_orientation.positionVector() - desired_effector_position_orientation.positionVector()).norm();
      
      if (pos_error < 0.01 && ori_error < 0.001) {
        xSemaphoreGive(RobotIdleSemaphore);
      }
      bool IK_succ = robot.goTo(desired_effector_position_orientation);

      thetas = robot.getJointAngles();
      xQueueSend(thetas_Queue, &thetas, 0);
      xQueueSend(IK_succ_Queue, &IK_succ, 0);

    } else if (settings.mode == control_CMD) {
      PoseCommand cmd;
      if (interpolator.hasNext()) {
        bool IK_succ = false;
        if (xSemaphoreTake(collisionSemaphore, 0) == pdTRUE) {
          interpolator.reset();
          xQueueReset(CMD_Queue);
        } else {
          IK_succ = robot.goTo(interpolator.next());
          if (!IK_succ) {
            interpolator.reset();
            xQueueReset(CMD_Queue);
          }
        }
        thetas = robot.getJointAngles();
        xQueueSend(thetas_Queue, &thetas, 0);
        xQueueSend(IK_succ_Queue, &IK_succ, 0);
      } else if (xQueueReceive(CMD_Queue, &cmd, 0)) {
        positionOrientation target_effector_position_orientation(cmd.x, cmd.y, cmd.z, cmd.roll, cmd.pitch, cmd.yaw);
        if (cmd.type == PoseCommand::Type::MOVE) {
          bool IK_succ = robot.goTo(target_effector_position_orientation);
          thetas = robot.getJointAngles();
          xQueueSend(thetas_Queue, &thetas, 0);
          xQueueSend(IK_succ_Queue, &IK_succ, 0);
        } else if (cmd.type == PoseCommand::Type::LINEAR) {
          interpolator.init(effector_position_orientation, target_effector_position_orientation, cmd.velocity, Ts);
        }
      }
    }

    xQueueSend(UI_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), 0);
    xQueueSend(sensor_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), 0);
    xQueueSend(data_stream_thetas_Queue, &thetas, 0);
    xQueueSend(data_stream_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), 0);
  }
}
void vTimerControlCallback(TimerHandle_t xTimer) {
  xSemaphoreGive(ControlSemaphore);
}

#include "src\VL53L1X\VL53L1X.h"

void TaskSensor(void* pvParameters) {


  const float Ts = 0.1;
  TwoWire Wire3(PB4, PA8);
  Wire3.begin();
  Wire3.setClock(400000);
  VL53L1X sensor_TOF;
  sensor_TOF.setBus(&Wire3);

  sensor_TOF.init();
  sensor_TOF.setMeasurementTimingBudget((uint32_t)(0.8 * Ts * 1000.0));
  sensor_TOF.setDistanceMode(VL53L1X::Short);
  sensor_TOF.setTimeout((uint16_t)(Ts * 1.5 * 1000.0));
  sensor_TOF.startContinuous((uint32_t)(Ts * 1000.0));

  vTaskDelay(pdMS_TO_TICKS(3000));

  positionOrientation effector_position_orientation;
  xQueueReceive(sensor_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), pdMS_TO_TICKS(1000));

  SensorTimer = xTimerCreate("TimerSensor",
                              pdMS_TO_TICKS(Ts * 1e3),
                              pdTRUE,
                              (void*)0,
                              vTimerSensorCallback);

  xTimerStart(SensorTimer, 0);

  const float max_range_m = 0.4f;

  while (1) {
    xSemaphoreTake(SensorSemaphore, portMAX_DELAY);

    uint16_t dist_mm = sensor_TOF.read();
    float dist_m = (float)(dist_mm * 1e-3f);
    xQueueReceive(sensor_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), 0);
    if (~sensor_TOF.timeoutOccurred() && dist_m < max_range_m) {
      if (dist_m < 0.035) xSemaphoreGive(collisionSemaphore);
      else xSemaphoreGive(nocollisionSemaphore);
      Matrix<3, 1> point_effector_frame;
      point_effector_frame << 0.0, 0.0, dist_m;
      Matrix<3, 1> T = effector_position_orientation.positionVector();
      Matrix<3, 3> R = effector_position_orientation.toRotationMatrix();
      T = Matrix<3, 1>::Zero();
      Matrix<3, 1> point_global_frame = T + point_effector_frame;
      xQueueSend(sensor_point_Queue, &point_global_frame, 0);
    }
  }
}

void vTimerSensorCallback(TimerHandle_t xTimer) {
  xSemaphoreGive(SensorSemaphore);
}

void TaskCMD(void* pvParameters) {

  int SERIAL_BUFFER_SIZE = 2048;
  char serialBuffer[SERIAL_BUFFER_SIZE];

  while (1) {
    if (settings.mode == control_CMD) {
      int len = Serial.readBytesUntil('\n', serialBuffer, SERIAL_BUFFER_SIZE - 1);
      if (len > 0) {
        serialBuffer[len] = '\0';  // null-terminate
        PoseCommand cmd;
        ParseStatus status = CommandParser::parse(serialBuffer, cmd);

        if (status == ParseStatus::OK) {
          xQueueSend(CMD_Queue, &cmd, 0);
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}



void TaskCom(void* pvParameters) {
  data_stream_server_instance data_stream_serial;
  data_stream_server_init(&data_stream_serial, data_stream_data_read_impl, data_stream_data_write_impl, serial_stream_read_fcn, serial_stream_write_fcn, (void*)&Serial, get_timestamp_impl, NULL, NULL);

  while (1) {
    data_stream_server(&data_stream_serial);
  }
}

data_stream_return_t data_stream_data_read_impl(data_stream_id_t stream_id, uint8_t data_size, void* data_ptr, void* user_data) {

  if (stream_id == 1) {
    Matrix<DOF, 1> thetas = Matrix<DOF, 1>::Zero();
    xQueueReceive(data_stream_thetas_Queue, &thetas, portMAX_DELAY);
    memcpy(data_ptr, thetas.data(), sizeof(float) * DOF);
    return data_stream_ok;
  } else if (stream_id == 2) {
    positionOrientation effector_position_orientation;
    xQueueReceive(data_stream_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), portMAX_DELAY);
    memcpy(data_ptr, reinterpret_cast<void*>(&effector_position_orientation), sizeof(positionOrientation));
    return data_stream_ok;
  } else {
    return data_stream_wrong_stream_params;
  }
}


data_stream_return_t data_stream_data_write_impl(data_stream_id_t stream_id, uint8_t data_size, const void* data_ptr, void* user_data) {

  return data_stream_wrong_stream_params;
}

data_stream_timestamp_t get_timestamp_impl(void* unused) {
  return millis() * 1e-3;
}


size_t serial_stream_write_fcn(void* handle, const void* buffer, size_t size, uint32_t timeout) {
  HardwareSerial* S = (HardwareSerial*)handle;
  size_t written = S->write((char*)buffer, size);
  S->flush();
  return written;
}
size_t serial_stream_read_fcn(void* handle, void* buffer, size_t size, uint32_t timeout) {
  HardwareSerial* S = (HardwareSerial*)handle;
  return S->readBytes((char*)buffer, size);
}
