#undef B1
#undef B0
#undef F

#include "Eigen/Dense"
#include "AeroShield.h"
#include "src\data_stream\data_stream.h"
#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"

// Type alias for convenience
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

// Min/max macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

// Include project headers
#include "LQ.h"
#include "Kalman.h"
#include "C2D.h"


SemaphoreHandle_t SampleSemaphore;
QueueHandle_t comQueue;

typedef struct
{
float y;
float y_ref;
float u;
}controlSignals;

// Sampling time
const float Ts = 0.05;

void setup() {
  Serial.begin(250000);
  delay(500);


 SampleSemaphore = xSemaphoreCreateBinary();
 comQueue = xQueueCreate(10, sizeof(controlSignals));

 AeroShield.sample_callback=SampleCallback;
 while (!AeroShield.begin(Ts)) {}


  xTaskCreate(TaskControl,
              "Control",
              10000,
              NULL,
              tskIDLE_PRIORITY + 3,
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

// Feedback gains
Matrix<1, 3> Kx;
Matrix<1, 1> Kz;

// System parameters
Matrix<4, 1> theta;
Matrix<3, 3> A;
Matrix<3, 1> B;
Matrix<1, 3> C;

// TODO C << 

{
  // System identification parameters
  theta << 1.4060, 11.0669, 7.8944, 0.0735;
  float K = theta(0);
  float a = theta(1);
  float omega_0 = theta(2);
  float b = theta(3);

  // Continuous-time system dynamics
  Matrix<3, 3> Ac;
  // TODO Ac << 

  Matrix<3, 1> Bc;
  // TODO Bc << 

  // Discretize the system
  C2D<3, 1>(A, B, Ac, Bc, Ts);
}

  // Augment the system for integral action
  Matrix<4, 4> A_tilde = Matrix<4, 4>::Zero();
  Matrix<4, 1> B_tilde = Matrix<4, 1>::Zero();
  Matrix<1, 4> C_tilde = Matrix<1, 4>::Zero();


  // TODO

  // LQR design
  Matrix<4, 4> Q_LQ;
  // TODO Q_LQ << 

  Matrix<1, 1> R_LQ;
  // TODO R_LQ << ;

  Matrix<4, 4> P_LQ;
  Matrix<1, 4> K_LQ;

  LQ::DARE<4, 1>(P_LQ, K_LQ, A_tilde, B_tilde, Q_LQ, R_LQ);

  // Split gain into state and integral part
  // TODO Kx =
  // TODO Kz = 


  // Noise covariance matrices
  Matrix<1, 1> R;
  R << 3;

  Matrix<3, 3> Q;
  Q << 10,  0,  0,
        0, 10,  0,
        0,  0, 50;

  // Initial control input
  Matrix<1, 1> u;
  u(0, 0) = 0;

  // Kalman filter instantiation
  KalmanFilter<3, 1, 1> KF(A, B, C, R, Q, Matrix<3, 3>::Zero(), Matrix<3, 1>::Zero());

  Matrix<1, 1> y_ref;
  y_ref(0, 0) = 0;

  Matrix<1, 1> y;

  Matrix<1, 1> z;
  z(0, 0) = 0; // Integral state


  while (1) {

    xSemaphoreTake(SampleSemaphore, portMAX_DELAY);
    // Wait for sensor sample
    while (!AeroShield.isSampleAvailable()) {}
    y(0, 0) = AeroShield.sensorReadDegreeSampled();

    // Reference update
    y_ref(0, 0) = AeroShield.referenceRead();

    // Feedback control law: u = -Kx*x - Kz*z
    // TODO u = 

    // Clamp output to actuator limits
    u(0, 0) = MAX(u(0, 0), 0);
    u(0, 0) = MIN(u(0, 0), 100);

    // Actuate
    AeroShield.actuatorWrite(u(0, 0));

    // Update integral state
    // TODO z += 
    
    controlSignals log;
    log.y=y(0,0);
    log.u=u(0,0);
    log.y_ref=y_ref(0,0);
    xQueueSend(comQueue, &log, 0);
  
  }

}

void loop() {

}

void SampleCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SampleSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
  } else {
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

