#pragma once

#include <Arduino.h>

#include <AccelStepper.h>

struct PinConfiguration {
  const int left_motor_step;
  const int left_motor_dir;
  const int left_sensor;

  const int right_motor_step;
  const int right_motor_dir;
  const int right_sensor;

  const int magnet;
};

class Delta {
public:
  // Delta();
  Delta(PinConfiguration pin_configuration);

  void setup();

  void home();

  void goTo(float x, float y);
  float getRightSliderPosition();
  float getLeftSliderPosition();
  void magnetOn();
  void magnetOff();

private:
  PinConfiguration pin_configuration_;
  AccelStepper stepper_left_;
  AccelStepper stepper_right_;

  bool is_homed_ = false;
  void dangersoursMoveRight();
  void dangerousMoveLeft();
  void moveTo(int left_position, int right_position);
  int mmPositionToRightStepper(float mm_position);
  int mmPositionToLeftStepper(float mm_position);
};