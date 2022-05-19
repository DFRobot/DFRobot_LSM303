/*!
 * @file magneticRawData.ino
 * @brief DFRobot's Manentic Sensor,This program continuously reads the accelerometer and magnetometer, 
 * @n     communicating the readings over the serial interface. You can display the readings with the Arduino Serial Monitor.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2021-5-05
 * @url https://github.com/DFRobot/DFRobot_LSM303
 */
#include <DFRobot_LSM303.h>

DFRobot_LSM303 compass;

char report[120];

void setup()
{
  Serial.begin(9600);
  compass.init();
  compass.enable();
}

void loop()
{
  compass.read();
  snprintf(report, sizeof(report), "Acceleration:(X:%6d Y:%6d Z:%6d) Magnetometer:(X:%6d Y:%6d Z:%6d)",
  compass.accelerometer.x, compass.accelerometer.y, compass.accelerometer.z,
  compass.magnetometer.x, compass.magnetometer.y, compass.magnetometer.z);
  Serial.println(report);
  delay(500);
}