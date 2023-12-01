#include <Arduino.h>
#include "NeuralNetwork.h"
#include "rplidar_driver_impl.h"
#include "VescUart-master/src/VescUart.h"
#include <ESP32Servo.h>
#include "lidar_data.h"
//#define DATA_COLLECTION

#define RPLIDAR_MOTOR D0

#ifndef DATA_COLLECTION
#define servoPin D1
HardwareSerial MySerial1(1);
Servo myservo;
VescUart vesc;
NeuralNetwork *nn;
const int minpos = 50;
const int maxpos = 130;
int pos = 0;
// float current = 1.0; /** The current in amps */
int rpm = 12700;
#else
#endif

RPLidar lidar;
const int MAX_NUM_POINTS = 360;
float distances[MAX_NUM_POINTS];
char report[80];
int point_count = 0;
rplidar_response_measurement_node_hq_t nodes[360];
size_t nodeCount = 360; // variable will be set to number of received measurement by reference

unsigned long startTime;

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  const float run = in_max - in_min;
  if (run == 0)
  {
    log_e("map(): Invalid input range, min == max");
    return -1; // AVR returns -1, SAM returns 0
  }
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (float)((delta * rise) / run + out_min);
}

void printSampleDuration()
{
  rplidar_response_sample_rate_t sampleInfo;
  lidar.getSampleDuration_uS(sampleInfo);

  snprintf(report, sizeof(report), "TStandard: %d[us] TExpress: %d[us]", sampleInfo.std_sample_duration_us, sampleInfo.express_sample_duration_us);
  Serial.println(report);
  delay(1000);
}

void setup()
{
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  lidar.begin();
  delay(1000);
  digitalWrite(RPLIDAR_MOTOR, HIGH); // turn on the motorz

  memset(distances, 0, MAX_NUM_POINTS*sizeof(float));
  printSampleDuration();

  Serial.begin(115200); // TODO  what to do this?

#ifndef DATA_COLLECTION
  MySerial1.begin(115200, SERIAL_8N1, D9, D10);
  vesc.setSerialPort(&MySerial1);
  // pinMode(servoPin, OUTPUT);
  myservo.attach(servoPin);

  nn = new NeuralNetwork();

  // float *inp_ptr = nn->getInputBuffer();
  // float *out_ptr = nn->getOutput();
  // for(int  j = 0; j < 10; j++){

  //     for(int i = 0; i< 270; i++)
  //       inp_ptr[i] = LidarData[j][i];
  //     nn->predict();


  //     float dir = out_ptr[0];
  //     int servo_val = (float)map(dir, -.35, .35, 120., 60.); 
  //     float speed = out_ptr[1];
  //     int rpm = map(speed, -1.0, 1.0, 3500., 5500.);
  //     Serial.printf("Servos: %f %d\n", dir, servo_val);
  // }


#endif
  delay(1000);
}

void loop()
{
  startTime = millis();

  if (!lidar.isScanning())
  {
    Serial.println("Not scanning");
    lidar.startScanNormal(true);
    analogWrite(RPLIDAR_MOTOR, 255); // turn on the motor
    //delay(10);
  }
  else
  {
    // loop needs to be send called every loop
    memset(distances, 0, MAX_NUM_POINTS*sizeof(float));
    for (int i = 0; i < 360; ++i)
    {
      if (IS_FAIL(lidar.loopScanData()))
      {
        Serial.println("Loop scan data failed");
      }
    }
    // create object to hold received data for processing

    u_result ans = lidar.grabScanData(nodes, nodeCount);
    if (IS_OK(ans))
    {
      // Serial.println("ans is ok");
      float distance_in_meters, angle;
      int idx;
      for (size_t i = 0; i < nodeCount; ++i)
      {
        // convert to standard units
        distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
        if (distance_in_meters > 0.15 && distance_in_meters < 4.0)
        {
          angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
          idx = (int)roundf(angle);
          distances[idx % MAX_NUM_POINTS] = distance_in_meters;
        }
        // snprintf(report, sizeof(report), "%.2f %.2f %d", distance_in_meters, angle_in_degrees, nodes[i].quality);
        // Serial.println(report);
      }
    }

#ifdef DATA_COLLECTION
    Serial.print("newscan");
    byte *p = (byte *)distances;
    //for (int i = 0; i < sizeof(distances); i++)
    for (int i = 45; i < MAX_NUM_POINTS - 45; ++i)
    {
      for(int j=0; j<sizeof(float); ++j)
        Serial.write(p[i*sizeof(float)+j]);
    }
  }
#else
  }
  float *inp_ptr = nn->getInputBuffer();

  for (int i = 45, j = 0; i < MAX_NUM_POINTS - 45 && j < MAX_NUM_POINTS-90; ++i, j++)
  {
    inp_ptr[j] = distances[i];
  }
  nn->predict();
  float *out_ptr = nn->getOutput();

  float dir = out_ptr[0];
  // int servo_val = (int) ((100. - (map(dir,-.35, .35, 50., 130.) - 50.))+ 50.);
  int servo_val = (int)map(dir, -.35, .35, 150., 30.); // 130 50 -->works
  // int servo_val = (float) map(dir,-.35, .35, 130., 0.);
  float speed = out_ptr[1];
  //int rpm = map(speed, -1.0, 1.0, 5000., 7000.);
  int rpm = map(speed, 0.0, 1.0, 9000., 12000.);
  // Serial.printf("Servo Value: %f %f\n", out_ptr[0], out_ptr[1]);
  Serial.printf("Servos: %f %d\n", dir, servo_val);

  myservo.write(servo_val);
  vesc.setRPM(rpm);

#endif

  unsigned long elapsedTime = millis() - startTime;
  Serial.println(elapsedTime);
}
