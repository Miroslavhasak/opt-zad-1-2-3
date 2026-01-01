
#undef B1
#undef B0
#undef F

#include "Eigen/Dense"
#include "FloatShield.h"
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


data_stream_timestamp_t get_timestamp_impl(void*);
data_stream_server_instance data_stream_serial;


SemaphoreHandle_t SampleSemaphore;
QueueHandle_t comQueue;

typedef struct
{
float y;
float y_ref;
float u;
}controlSignals;


const float Ts=0.05;

void setup() {              
  Serial.begin(250000);   
  Wire.begin();  
  Wire.setClock(400000);

  FloatShield.sample_callback=SampleCallback;
SampleSemaphore = xSemaphoreCreateBinary();
comQueue = xQueueCreate(3, sizeof(controlSignals));

   while(!FloatShield.begin(Ts)){}
  analogWriteResolution(10);
  analogReadResolution(10);

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

void loop() {
}


void SampleCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SampleSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void TaskControl(void* pvParameters) { 


// Feedback gains
Matrix<1, 3> Kx;
Matrix<1, 1> Kz;

// System parameters
Matrix<4, 1> theta;

Matrix<3, 3> A=Matrix<3, 3>::Zero();
Matrix<3, 1> Bu=Matrix<3, 1>::Zero();
Matrix<3, 1> Bg=Matrix<3, 1>::Zero();
Matrix<1, 3> C=Matrix<1, 3>::Zero();

C << 0, 0, 1;

{
  // System identification parameters
  theta << 0.0116  ,  2.0895  ,  0.2959  ,  0.045;
  float K = theta(0);
  float a = theta(1);
  float b = theta(2);
  float m = theta(3);

  // Continuous-time system dynamics
  Matrix<3, 3> Ac;
  Ac << -a,  0, 0,
         K/m, -b/m, 0,
         0,  1, 0;

  Matrix<3, 1> Buc;
  Buc << a, 0, 0;

  
  Matrix<3, 1> Bgc;
  Bgc << 0, 1, 0;

  Matrix<2,2> Acblock = Ac.block<2,2>(0,0);
  Matrix<2,1> Bucblock = Buc.block<2,1>(0,0);
  Matrix<2,1> Bgcblock = Bgc.block<2,1>(0,0);
  Matrix<2,2> Ablock;
  Matrix<2,1> Bublock;
  Matrix<2,1> Bgblock;

  C2D<2, 1>(Ablock, Bgblock, Acblock , Bgcblock , Ts);
  C2D<2, 1>(Ablock, Bublock, Acblock , Bucblock , Ts);

  A.block<2,2>(0,0)=Ablock;
  Bu.block<2,1>(0,0)=Bublock;
  Bg.block<2,1>(0,0)=Bgblock;

  A(2,1)=Ts;
  A(2,2)=1;

}

  // Augment the system for integral action
  Matrix<4, 4> A_tilde = Matrix<4, 4>::Zero();
  Matrix<4, 1> B_tilde = Matrix<4, 1>::Zero();
  Matrix<1, 4> C_tilde = Matrix<1, 4>::Zero();

  //TODO
{
  // LQR design
  Matrix<4, 4> Q_LQ;
  // TODO Q_LQ << 
 
  Matrix<1, 1> R_LQ;
  //TODO R_LQ << 

  Matrix<4, 4> P_LQ;
  Matrix<1, 4> K_LQ;

  LQ::DARE<4, 1>(P_LQ, K_LQ, A_tilde, B_tilde, Q_LQ, R_LQ);


  // Split gain into state and integral part
  //TODO Kx = 
  //TODO Kz =
}
  // Noise covariance matrices
  Matrix<1, 1> R;
  //TODO R << 

  Matrix<3, 3> Q;

  //TODO Q <<   

  Matrix<1, 1> u;
  u(0, 0) = 0;


  // Kalman filter instantiation
  Matrix<3, 2> B;
  B.block(0, 0, 3, 1)=Bu;
  B.block(0, 1, 3, 1)=Bg;

  KalmanFilter<3, 2, 1> KF(A, B, C, R, Q, Matrix<3, 3>::Zero(), Matrix<3, 1>::Zero());

  Matrix<1, 1> y_ref;
  y_ref(0, 0) = 0;

  Matrix<1, 1> y;
  Matrix<1, 1> z;
  z(0, 0) = 0; // Integral state
  

  while (1) {

    xSemaphoreTake(SampleSemaphore, portMAX_DELAY);
    // Wait for sensor sample
    while (!FloatShield.isSampleAvailable()) {}
    y(0, 0) = FloatShield.sensorReadAltitudeSampled()/1000.0;

    // Reference update
    y_ref(0, 0) = FloatShield.referenceRead()*0.0025;

    Matrix<2, 1> u_;
    u_<<u,-9.81;

    //TODO u = 

    // Clamp output to actuator limits
    u(0, 0) = MAX(u(0, 0), 0);
    u(0, 0) = MIN(u(0, 0), 100);

    // Actuate
    FloatShield.actuatorWrite(u(0, 0));

    // Update integral state
    //TODO z =

    // Update integral state
    controlSignals log;
    log.y=y(0,0);
    log.u=u(0,0);
    log.y_ref=y_ref(0,0);
    xQueueSend(comQueue, &log, 0);
  
  }

}


void TaskCom(void* pvParameters) {
  data_stream_server_instance data_stream_serial;
  data_stream_server_init(&data_stream_serial, data_stream_data_read_impl, data_stream_data_write_impl, serial_stream_read_fcn, serial_stream_write_fcn, (void*)&Serial, get_timestamp_impl, NULL, NULL);

  while (1) {
    data_stream_server(&data_stream_serial);
  }
}



data_stream_return_t data_stream_data_read_impl(data_stream_id_t stream_id, uint8_t data_size, void* data_ptr,void* user_data) {

if(stream_id==1)
{
    controlSignals val;
    xQueueReceive(comQueue, &val, portMAX_DELAY);
    memcpy(data_ptr, &val, sizeof(val));
   return data_stream_ok;
}
else
{
   return data_stream_wrong_stream_params;
}

}


data_stream_return_t data_stream_data_write_impl(data_stream_id_t stream_id, uint8_t data_size, const void* data_ptr,void* user_data) {

   return data_stream_wrong_stream_params;

}


data_stream_timestamp_t get_timestamp_impl(void* unused)
{
  return  FloatShield.getTick()*Ts;
}



size_t serial_stream_write_fcn(void* handle, const void* buffer, size_t size,uint32_t timeout)
{
  HardwareSerial* S = (HardwareSerial*)handle;
  size_t written= S->write((char*)buffer, size); 
  S->flush();
  return written;
}
size_t serial_stream_read_fcn(void* handle, void* buffer, size_t size,uint32_t timeout)
{
  HardwareSerial* S = (HardwareSerial*)handle;
  return S->readBytes((char*)buffer, size);
}
