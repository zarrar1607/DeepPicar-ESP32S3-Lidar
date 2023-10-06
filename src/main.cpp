#include <Arduino.h>
#include "NeuralNetwork.h"
#include "RPLidar.h"
#include <HardwareSerial.h>
#define RPLIDAR_MOTOR D0
HardwareSerial MySerial0(0);
RPLidar lidar;
NeuralNetwork *nn;
const int NUM_POINTS = 360;
float distances[NUM_POINTS];

static int count = 0;
unsigned long startTime;

const int num_lidar_range_values = 1081;
void setup()
{
  Serial.begin(115200);
  lidar.begin(MySerial0);
  Serial.println("Lidar Readly");
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  delay(10000);

  nn = new NeuralNetwork();

  delay(10000);
}
void loop()
{
  Serial.println("Done Allocating");

  if (IS_OK(lidar.waitPoint()))
  {
    if (count == 0)
    {
      startTime = millis();
    }
    // Serial.println("Getting Data");
    float distance = lidar.getCurrentPoint().distance; // distance value in mm unit
    float angle = lidar.getCurrentPoint().angle;       // anglue value in degree
    bool startBit = lidar.getCurrentPoint().startBit;  // whether this point is belong to a new scan
    byte quality = lidar.getCurrentPoint().quality;    // quality of the current measurement

    Serial.print("Distance (mm): ");
    Serial.println(distance);

    Serial.print("Angle (degrees): ");
    Serial.println(angle);

    // Map angle to array index
    int index = int(angle) % NUM_POINTS;
    // Store the distance at the corresponding angle index
    distances[index] = distance;
    count++; // Increment the point count

    // Check if we have received all 360 points
    if (count == NUM_POINTS)
    {
      unsigned long elapsedTime = millis() - startTime;
      // Print the array of distances
      Serial.println("Distances array:");
      for (int i = 0; i < NUM_POINTS; i++)
      {
        Serial.print("Index ");
        Serial.print(i);
        Serial.print(", Angle ");
        Serial.print((360.0 / NUM_POINTS) * i); // Angle corresponding to the index
        Serial.print(" degrees, Distance ");
        Serial.println(distances[i]);
      }
      Serial.print("Time for one full scan: ");
      Serial.print(elapsedTime);
      Serial.println(" ms");
      // Reset the count for the next round
      count = 0;


      
      float *inp_ptr = nn->getInputBuffer();
      for (int i = 0; i < num_lidar_range_values; ++i)
      {
        float randomValue = random(0, 91); // Generates a random float between 0 and 90
        inp_ptr[i] = randomValue;
      }
      unsigned long startTimeNN = millis(); // Record the start time
      nn->predict();
      unsigned long endTimeNN = millis(); // Record the end time

      unsigned long executionTimeNN = endTimeNN - startTimeNN;
      Serial.print("Prediction Time: ");
      Serial.print(executionTimeNN);
      Serial.println(" milliseconds");

      float *out_ptr = nn->getOutput();
      Serial.printf("%f %f\n", out_ptr[0], out_ptr[1]);

      delay(10000);
    }
  }
  else
  {
    Serial.println("Stop Motor");
    analogWrite(RPLIDAR_MOTOR, 0);
    rplidar_response_device_info_t info;
    Serial.println("Detecting");
    if (IS_OK(lidar.getDeviceInfo(info, 100)))
    {
      Serial.println("Found it");
      lidar.startScan();

      Serial.println("Start Motor");
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(5000);
    }
  }
}
