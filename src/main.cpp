#include <Arduino.h>
#include "NeuralNetwork.h"
#include <RPLidar.h>
#include <HardwareSerial.h>
#define RPLIDAR_MOTOR 10
HardwareSerial MySerial(0);
RPLidar lidar;
NeuralNetwork *nn;

const int num_lidar_range_values = 1081;
void setup()
{
  Serial.begin(115200);
  MySerial.begin(115200);
  lidar.begin(MySerial);
  Serial.println("Readly");
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  delay(10000);

  nn = new NeuralNetwork();

  delay(10000);
}
void loop()
{
  Serial.println("Done Allocating");
  float* inp_ptr = nn->getInputBuffer();
  for (int i = 0; i < num_lidar_range_values; ++i){
    float randomValue = random(0, 91); // Generates a random float between 0 and 90
    inp_ptr[i] = randomValue;
  }
  unsigned long startTime = millis(); // Record the start time
  nn->predict();
  unsigned long endTime = millis();   // Record the end time

  unsigned long executionTime = endTime - startTime;
  Serial.print("Prediction Time: ");
  Serial.print(executionTime);
  Serial.println(" milliseconds");

  float* out_ptr = nn->getOutput();
  Serial.printf("%f %f\n", out_ptr[0], out_ptr[1]);

  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    
    Serial.printf("Angle %f : %f\n", angle, distance);
    
    
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       //delay(1000);
    }
  }
  //delay(1000);
}
