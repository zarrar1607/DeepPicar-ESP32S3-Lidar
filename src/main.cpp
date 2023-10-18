#include <Arduino.h>
#include "NeuralNetwork.h"
#include "rplidar_driver_impl.h"

#define RPLIDAR_MOTOR D0

RPLidar lidar;
NeuralNetwork *nn;
const int MAX_NUM_POINTS = 360;
float distances[MAX_NUM_POINTS];
char report[80];
int point_count = 0;
rplidar_response_measurement_node_hq_t nodes[60];
size_t nodeCount = 60; // variable will be set to number of received measurement by reference

unsigned long startTime;

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
  Serial.begin(115200);
  digitalWrite(RPLIDAR_MOTOR, HIGH); // turn on the motorz

  // nn = new NeuralNetwork();

  memset(distances, 0, MAX_NUM_POINTS);
  printSampleDuration();
  delay(1000);
}

void loop()
{
  startTime = millis();

  if (!lidar.isScanning())
  {
    Serial.println("Not scanning");
    lidar.startScanNormal(true);
    digitalWrite(RPLIDAR_MOTOR, HIGH); // turn on the motor
    delay(10);
  }
  else
  {
    // loop needs to be send called every loop
    for (int i = 0; i < 60; ++i)
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
        angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
        distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
        idx = (int)roundf(angle);
        distances[idx % MAX_NUM_POINTS] = distance_in_meters;
        // snprintf(report, sizeof(report), "%.2f %.2f %d", distance_in_meters, angle_in_degrees, nodes[i].quality);
        // Serial.println(report);
      }
    }

    Serial.print("newscan");
    byte *p = (byte *)distances;
    for (int i = 0; i < sizeof(distances); i++)
    {
      Serial.write(p[i]);
    }
  }

  // unsigned long elapsedTime = millis() - startTime;
  // Serial.println(elapsedTime);
}
