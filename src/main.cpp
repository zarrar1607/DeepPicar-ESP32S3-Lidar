#include <Arduino.h>
#include "NeuralNetwork.h"
NeuralNetwork *nn;

const int num_lidar_range_values = 1081;
float inputBuffer[1081];
void setup()
{
  Serial.begin(115200);
  Serial.println("Readly");

  for (int i = 0; i < num_lidar_range_values; ++i)
    inputBuffer[i] = 0;

  

  delay(10000);

  nn = new NeuralNetwork();

  delay(10000);
}
void loop()
{
  Serial.println("Done Allocating");
  delay(20000);
}
