/*!
 * @file navigationAngleRead.ino
 * @brief DFRobot's Manentic Sensor,This program continuously reads the accelerometer and magnetometer, 
 * @n     communicating the readings over the serial interface. You can display the readings with the Arduino Serial Monitor.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT license (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2021-5-05
 * @url https://github.com/DFRobot/DFRobot_LSM303
 */
#include <DFRobot_Lsm303.h>

DFRobot_Lsm303 compass;

void setup() {
  Serial.begin(9600);
  compass.init();
  compass.enable();
}

void loop() {
  compass.read();
  float heading = compass.getNavigationAngle();
  Serial.print("Navigation Angle:  ");
  Serial.println(heading,3);
  delay(500);  // delay for serial readability
}