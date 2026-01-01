#undef B1
#undef B0
#undef F

#include "Eigen/Dense"
#include "ThermalShield.h"
#include "src\data_stream\data_stream.h"
#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"

// Type alias for convenience
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

// Min/max macros
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

// Include project headers
#include "Kalman.h"
#include "C2D.h"
#include "MPC.h"
#include "quadprog.h"

const float Ts = 10;

SemaphoreHandle_t ControlSemaphore;
QueueHandle_t comQueue;

typedef struct
{
float y;
float y_ref;
float u;
}controlSignals;


void setup(void) {

  Serial.begin(250000);
  delay(1000);

  ControlSemaphore = xSemaphoreCreateBinary();
  comQueue = xQueueCreate(10, sizeof(controlSignals));

  ThermalShield.begin(Ts);
  ThermalShield.sample_callback = SampleCallback;
  ThermalShield.second_callback = SecondCallback;

  xTaskCreate(TaskControl,
              "Control",
              25000,
              NULL,
              tskIDLE_PRIORITY + 3,
              NULL);

  xTaskCreate(TaskDisplay,
              "Display",
              32,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);

  xTaskCreate(TaskCom,
              "Com",
              256,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);

  vTaskStartScheduler();
}

void TaskControl(void* pvParameters) {

  Matrix<4, 4> A_tilde = Matrix<4, 4>::Zero();
  Matrix<4, 1> B_tilde = Matrix<4, 1>::Zero();
  Matrix<1, 4> C_tilde = Matrix<1, 4>::Zero();
  {
    // System parameters
    Matrix<4, 1> theta;
    Matrix<3, 3> A;
    Matrix<3, 1> B;
    Matrix<1, 3> C;
    // TODO C << 

    // System identification parameters
    theta << 1.1896, 0.0248, 0.0008, 0.0875;
    float K = theta(0);
    float a1 = theta(1);
    float a2 = theta(2);
    float a3 = theta(3);

    // Continuous-time system dynamics
    Matrix<3, 3> Acont;
    // TODO Acont << 

    Matrix<3, 1> Bcont;
    // TODO Bcont << 

    // Discretize the system
    C2D<3, 1>(A, B, Acont, Bcont, Ts);

  }

  constexpr int p = 20;
  // Noise covariance matrices
  Matrix<1, 1> R;
  R << 0.1250;

  Matrix<4, 4> Q;
  Q << 10, 0, 0, 0,
    0, 10, 0, 0,
    0, 0, 10, 0,
    0, 0, 0, 100;


  float T0 = ThermalShield.sensorRead();
  Matrix<4, 1> x_hat0;
  x_hat0 << T0, T0, T0, 0;

  // Kalman filter instantiation
  KalmanFilter<4, 1, 1> KF(A_tilde, B_tilde, C_tilde, R, Q, Matrix<4, 4>::Zero(), x_hat0);


  Matrix<1, 1> y_ref;
  y_ref << 50;

  // Initial control input
  Matrix<1, 1> u;
  u(0, 0) = 0;

  Matrix<1, 1> Q_MPC;
  // TODO Q_MPC <<
  Matrix<1, 1> R_MPC;
  // TODO R_MPC <<


  Matrix<1, 1> u_max;
  u_max << 100;
  Matrix<1, 1> u_min;
  u_min << 0;



  while (1) {

    xSemaphoreTake(ControlSemaphore, portMAX_DELAY);

    while (!ThermalShield.isSampleAvailable()) {}

    Matrix<1, 1> y;
    y_ref << ThermalShield.referenceRead();
    y(0, 0) = ThermalShield.sensorReadSampled();

    // State estimation
   // TODO  Matrix<4, 1> x_hat =

    Matrix<p * 1, 1> b;
    Matrix<p * 1, p * 1> H;


   // TODO MPC::getHbIncrementalOutput<4, 1, 1, p>();


    Matrix<p * 2, p> Ac;
    Matrix<p * 2, 1> bc;

    // TODO MPC::getU_con_incremental<1, p>();


    Matrix<p * 1, 1> delta_U;

    // TODO quadprog<p, 2 * p>();
   // TODO u +=

    // Actuate
    ThermalShield.actuatorWrite(u(0, 0));
  }
}


void SampleCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(ControlSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void SecondCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  controlSignals log;
  log.y=ThermalShield.getLastTemp();
  log.u=ThermalShield.getLastU();
  log.y_ref=ThermalShield.getLastRef();
  xQueueSendFromISR(comQueue, &log,&xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void loop() {
}

void TaskDisplay(void* pvParameters) {

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    ThermalShield.displayPool();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
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
    controlSignals val;
    xQueueReceive(comQueue, &val, portMAX_DELAY);
    memcpy(data_ptr, &val, sizeof(val));
    return data_stream_ok;
  } 
  else {
    return data_stream_wrong_stream_params;
  }
}


data_stream_return_t data_stream_data_write_impl(data_stream_id_t stream_id, uint8_t data_size, const void* data_ptr, void* user_data) {

  return data_stream_wrong_stream_params;
}

data_stream_timestamp_t get_timestamp_impl(void* unused) {
  return millis() / 1000.0;
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
