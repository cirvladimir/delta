#include <Arduino.h>
#include <SoftwareSerial.h>

#include "delta.h"

PinConfiguration pin_config = {.left_motor_step = 3,
                               .left_motor_dir = 2,
                               .left_sensor = 8,
                               .right_motor_step = 5,
                               .right_motor_dir = 4,
                               .right_sensor = 7,
                               .magnet = 6};

Delta delta(pin_config);

void setup() {
  Serial.begin(115200);

  delta.setup();

  // delta.magnetOn();
  // delay(2000);
  // delta.magnetOff();
  // return;
  delta.home();
  Serial.println("done homing");

  delta.goTo(10.0, 0.0);
  delta.magnetOn();
  delay(200);

  delta.goTo(50.0, 10.0);
  delta.goTo(220.0, 10.0);

  delta.magnetOff();
  delay(200);

  delta.goTo(50.0, 10.0);
  delta.goTo(50.0, 0.0);

  delta.magnetOn();
  delay(200);

  delta.goTo(50.0, 10.0);
  delta.goTo(220.0, 10.0);

  delta.magnetOff();
  delay(200);

  delta.goTo(100.0, 10.0);
  delta.goTo(100.0, 0.0);

  delta.magnetOn();
  delay(200);

  delta.goTo(100.0, 10.0);
  delta.goTo(220.0, 10.0);

  delta.magnetOff();
  delay(200);

  delta.goTo(150.0, 10.0);
  delta.goTo(150.0, 0.0);

  delta.magnetOn();
  delay(200);

  delta.goTo(150.0, 10.0);
  delta.goTo(220.0, 10.0);

  delta.magnetOff();
}

void loop() {
  // if (delta.leftSensor()) {
  //   Serial.println("yes");
  // }
  // if (millis() % 4000 < 2000) {
  //   delta.moveRight();
  // } else {
  //   delta.moveLeft();
  // }
  // delta.moveRight();
  // Serial.print(delta.getLeftSliderPosition());
  // Serial.print(" - ");
  // Serial.print(delta.getRightSliderPosition());
  // Serial.println();
  // delay(500);
}
