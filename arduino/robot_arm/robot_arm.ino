
#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include <Eigen/Dense>

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "src\LiquidMenu\LiquidCrystal_I2C.h"
#include "src\LiquidMenu\LiquidMenu.h"
#include "src\Button.h"
#include "src\Joystick.h"
#include "src\Keypad\Keypad.h"
#include "src\data_stream\data_stream.h"
#include "command_parser.h"
#include "mirobotController.h"


#define USE_MIROBOT


template<int Rows, int Cols> using Matrix = Eigen::Matrix<float, Rows, Cols>;

#include "robot.h"
#include "interpolation.h"


TaskHandle_t HandleTaskControl;
TaskHandle_t HandleTaskUI;
TaskHandle_t HandleTaskCOM;
TaskHandle_t HandleTaskCMD;
SemaphoreHandle_t UISemaphore;
SemaphoreHandle_t ControlSemaphore;

TimerHandle_t UITimer;
TimerHandle_t ControlTimer;

QueueHandle_t UI_effector_position_orientation_Queue;
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
 

  UI_effector_position_orientation_Queue = xQueueCreate(1, sizeof(positionOrientation));
  desired_effector_position_orientation_Queue = xQueueCreate(1, sizeof(positionOrientation));
  IK_succ_Queue = xQueueCreate(1, sizeof(bool));
  thetas_Queue = xQueueCreate(1, sizeof(Matrix<DOF, 1>));
  CMD_Queue = xQueueCreate(30, sizeof(PoseCommand));
  data_stream_thetas_Queue = xQueueCreate(1, sizeof(Matrix<DOF, 1>));
  data_stream_effector_position_orientation_Queue = xQueueCreate(1, sizeof(positionOrientation));
  

  xTaskCreate(TaskControl,
              "Control",
              1500,
              NULL,
              tskIDLE_PRIORITY + 4,
              &HandleTaskControl);

  xTaskCreate(TaskUI,
              "UI",
              500,
              NULL,
              tskIDLE_PRIORITY + 2,
              &HandleTaskUI);

  /*
  xTaskCreate(TaskCom,
              "Com",
              500,
              NULL,
              tskIDLE_PRIORITY + 1,
              &HandleTaskCOM);
*/

  xTaskCreate(TaskCMD,
              "CMD",
              1000,
              NULL,
              tskIDLE_PRIORITY + 1,
              &HandleTaskCMD);


  vTaskStartScheduler();
}

void loop(void) {
}


void TaskUI(void* pvParameters) {

  const int update_rate=50;

  LiquidCrystal_I2C lcd(0x27, 16, 2);
  lcd.init();
  lcd.backlight();

  char keys[4][4] = {
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

#ifndef USE_MIROBOT
  desired_effector_position_orientation.X = 0.0;
  desired_effector_position_orientation.Y = 0.15;
  desired_effector_position_orientation.Z = 0.10;
  desired_effector_position_orientation.r = -PI;
  desired_effector_position_orientation.p = 0;
  desired_effector_position_orientation.y = 0;

#endif

#ifdef USE_MIROBOT
  desired_effector_position_orientation.X = 0.0;
  desired_effector_position_orientation.Y = 0.2;
  desired_effector_position_orientation.Z = 0.23;
  desired_effector_position_orientation.r = 0;
  desired_effector_position_orientation.p = 0;
  desired_effector_position_orientation.y = 0;

#endif
  Matrix<DOF, 1> thetas = Matrix<DOF, 1>::Zero();
  bool IK_succ = true;
#include "menu.h"

  UITimer = xTimerCreate("TimerUI",
                         pdMS_TO_TICKS(update_rate),
                         pdTRUE,
                         (void*)0,
                         vTimerUICallback);

  xTimerStart(UITimer, 0);

#ifndef USE_MIROBOT
 float rate = 1;
#endif

#ifdef USE_MIROBOT
 float rate = 0.15;
#endif


  while (1) {


    xSemaphoreTake(UISemaphore, portMAX_DELAY);
    xQueueReceive(UI_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), 0);

    joystick.read();

    char key = keypad.getKey();
    bool mode_change=false;

    if (key)
    {

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
        mode_change=true;
        settings.mode = static_cast<control_mode_t>((settings.mode + 1) % 4);
        if (settings.mode == control_thetas)
          strcpy(mode_string, "ang ");
        else if (settings.mode == control_position)
          strcpy(mode_string, "pos ");
        else if (settings.mode == control_orientation)
          strcpy(mode_string, "ori ");
        else if (settings.mode == control_CMD)
      {
          strcpy(mode_string, "cmd ");
          strcpy(aux_string, "");
      }    
      }
      menu.update();
    }

    if (button_variable.check() == LOW || mode_change) {

      if (settings.mode == control_thetas) {
        settings.joint_number = ((settings.joint_number + !mode_change) % DOF);
        snprintf(aux_string, sizeof(aux_string), "%d", settings.joint_number + 1);
      } else if (settings.mode == control_position) {
        settings.position_control = static_cast<position_pair_t>((settings.position_control + !mode_change) % 3);

        if (settings.position_control == XY) {
        strcpy(aux_string, "XY");
      } else if (settings.position_control == XZ) {
        strcpy(aux_string, "XZ");
      } else if (settings.position_control == YZ) {
        strcpy(aux_string, "YZ");
      }  
       
      } else if (settings.mode == control_orientation) {

         settings.orientation_control = static_cast<orientation_pair_t>((settings.orientation_control + !mode_change) % 3);
        if (settings.orientation_control == rp) {
        strcpy(aux_string, "rp ");
      } else if (settings.orientation_control == ry) {
        strcpy(aux_string, "ry ");
      } else if (settings.orientation_control == py) {
        strcpy(aux_string, "py ");
      }

      }
       menu.update();
    }

    if (settings.mode == control_thetas) {

      thetas(settings.joint_number) += 0.05 * joystick.getX();
      xQueueSend(thetas_Queue, &thetas, 0);
      desired_effector_position_orientation = effector_position_orientation;
    }

    else if (settings.mode == control_position) {
      
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
      xQueueReceive(thetas_Queue, &thetas, 0);
    } else if (settings.mode == control_orientation) {
    
      if (settings.orientation_control == rp) {
        desired_effector_position_orientation.r += 0.025* rate * joystick.getX();
        desired_effector_position_orientation.p += 0.025 * rate * joystick.getY();
      } else if (settings.orientation_control == ry) {
        desired_effector_position_orientation.r += 0.025 * rate * joystick.getX();
        desired_effector_position_orientation.y += 0.025 * rate * joystick.getY();
      } else if (settings.orientation_control == py) {
        desired_effector_position_orientation.p += 0.025 * rate * joystick.getX();
        desired_effector_position_orientation.y += 0.025 * rate * joystick.getY();
      }
      xQueueReceive(thetas_Queue, &thetas, 0);
    } else if (settings.mode == control_CMD) {
      xQueueReceive(thetas_Queue, &thetas, 0);
    }

    if (settings.mode == control_position || settings.mode == control_orientation || settings.mode == control_CMD) {
      xQueueReceive(IK_succ_Queue, &IK_succ, 0);
      if (!IK_succ) {
        desired_effector_position_orientation = effector_position_orientation;
        strcpy(aux_string, "IKf");
      }
      xQueueSend(desired_effector_position_orientation_Queue, reinterpret_cast<void*>(&desired_effector_position_orientation), 0);
    }
    menu.softUpdate();
  }
}

void vTimerUICallback(TimerHandle_t xTimer) {
  xSemaphoreGive(UISemaphore);
}


#include "ServoActuator.h"

void TaskControl(void* pvParameters) {

#ifndef USE_MIROBOT
  const float Ts = 0.025;
#endif
#ifdef USE_MIROBOT
  const float Ts = 0.1;
#endif


#ifndef USE_MIROBOT
TwoWire Wire2(PF0, PF1);
Wire2.begin();
Wire2.setClock(400000);

std::vector<actuator_parameters> servo_parameters;
servo_parameters.resize(6);
servo_parameters[0]={-6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0};
servo_parameters[1]={-5.0 / 10.0 * PI, 5.2 / 10.0 * PI, 2.0 / 3.0, 0.015};
servo_parameters[2]={-8.2 / 10.0 * PI, 5.0 / 10.0 * PI, -2.0 / 3.0, 0.12};
servo_parameters[3]={-7.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, -0.2};
servo_parameters[4]={-6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0.0};
servo_parameters[5]={-6.0 / 10.0 * PI, 6.0 / 10.0 * PI, -2.0 / 3.0, 0.0};

RobotServoController servo_controller(Wire2,servo_parameters);
servo_controller.init();

#endif

#ifdef USE_MIROBOT
HardwareSerial Serial1(PA10, PA9);
Serial1.begin(115200);	
std::vector<actuator_parameters> mirobot_parameters;
mirobot_parameters.resize(6);
mirobot_parameters[0]={-110.0*PI/180.0, 160.0*PI/180.0, 1.0, 0.0};
mirobot_parameters[1]={-70.0*PI/180.0,  35.0*PI/180.0, -1.0, 0.0};
mirobot_parameters[2]={-60.0*PI/180.0,  120.0*PI/180.0, -1.0, 0.0};
mirobot_parameters[3]={-180.0*PI/180.0, 180.0*PI/180.0, 1.0, 0.0};
mirobot_parameters[4]={-30.0*PI/180.0,  200.0*PI/180.0, -1.0, 0.0};
mirobot_parameters[5]={-360.0*PI/180.0, 360.0*PI/180.0, 1.0, 0.0};

MirobotController mirobot_controller(&Serial1,mirobot_parameters);
mirobot_controller.init();

#endif

RotationZ rotZ_obj;
RotationX rotX_obj;
RotationY rotY_obj;

#ifndef USE_MIROBOT

  Matrix<3, 1> T[DOF];
  /*
  T[0] << 
  T[1] << 
  T[2] << 
  T[3] << 
  T[4] << 
  T[5] << 
  */

  const RotationMatrix* R[DOF] = { &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj };
  Robot::RobotKinematics<DOF> robot_kinematics(R, T);
  Robot::RobotArm<DOF> robot(R, T, &servo_controller);

  #endif

  #ifdef USE_MIROBOT

  Matrix<3, 1> T[DOF];
/*
  T[0] << 
  T[1] << 
  T[2] << 
  T[3] << 
  T[4] << 
  T[5] << 
  */
  const RotationMatrix* R[DOF] = { &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj, &rotX_obj };
  Robot::RobotKinematics<DOF> robot_kinematics(R, T);
  Robot::RobotArm<DOF> robot(R, T, &mirobot_controller);
  
  #endif
  
 

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

  while (1) {

    xSemaphoreTake(ControlSemaphore, portMAX_DELAY);

    Matrix<3, 1> effector_position = robot.getCurrentEffectorPosition();
    Matrix<3, 3> R = robot.getCurrentOrientation();
    effector_position_orientation = positionOrientation(effector_position, R);


    if (settings.mode == control_thetas) {
      xQueueReceive(thetas_Queue, &thetas, 0);
      robot.setJointAngles(thetas);
    } else if (settings.mode == control_position || settings.mode == control_orientation) {
      xQueueReceive(desired_effector_position_orientation_Queue, reinterpret_cast<void*>(&desired_effector_position_orientation), 0);
      bool IK_succ = robot.goTo(desired_effector_position_orientation);
      thetas = robot.getJointAngles();
      xQueueSend(thetas_Queue, &thetas, 0);
      xQueueSend(IK_succ_Queue, &IK_succ, 0);

    } else if (settings.mode == control_CMD) {
      PoseCommand cmd;
      if (interpolator.hasNext()) {
        bool IK_succ=false;
          IK_succ = robot.goTo(interpolator.next());
          if (!IK_succ) {
            interpolator.reset();
            xQueueReset(CMD_Queue);
            Serial.println("IK fail");
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
    xQueueSend(data_stream_thetas_Queue, &thetas, 0);
    xQueueSend(data_stream_effector_position_orientation_Queue, reinterpret_cast<void*>(&effector_position_orientation), 0);
  }
}
void vTimerControlCallback(TimerHandle_t xTimer) {
  xSemaphoreGive(ControlSemaphore);
}


void TaskCMD(void* pvParameters) {

  int SERIAL_BUFFER_SIZE = 1024;
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
  }  else {
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
