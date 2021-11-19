#include "delta.h"

#include "geometry.h"

const int MICROSTEPS = 4;
const int STEPS_PER_ROTATION = 200;
const int SPEED = MICROSTEPS * STEPS_PER_ROTATION * 4;

Delta::Delta(PinConfiguration pin_configuration)
    : pin_configuration_(pin_configuration),
      stepper_left_(AccelStepper::DRIVER, pin_configuration.left_motor_step,
                    pin_configuration.left_motor_dir),
      stepper_right_(AccelStepper::DRIVER, pin_configuration.right_motor_step,
                     pin_configuration.right_motor_dir) {}

void Delta::setup() {
  pinMode(pin_configuration_.left_sensor, INPUT);
  pinMode(pin_configuration_.right_sensor, INPUT);
  pinMode(pin_configuration_.magnet, OUTPUT);
  digitalWrite(pin_configuration_.magnet, LOW);

  pinMode(pin_configuration_.belt_forward, OUTPUT);
  pinMode(pin_configuration_.belt_backward, OUTPUT);
  digitalWrite(pin_configuration_.belt_forward, LOW);
  digitalWrite(pin_configuration_.belt_backward, LOW);

  stepper_left_.setMaxSpeed(SPEED);
  stepper_right_.setMaxSpeed(SPEED);
}

void Delta::dangersoursMoveRight() {
  stepper_left_.setSpeed(SPEED);
  stepper_right_.setSpeed(-SPEED);
  stepper_left_.runSpeed();
  stepper_right_.runSpeed();
}

void Delta::dangerousMoveLeft() {
  stepper_left_.setSpeed(-SPEED);
  stepper_right_.setSpeed(SPEED);
  stepper_left_.runSpeed();
  stepper_right_.runSpeed();
}

void Delta::home() {
  while (digitalRead(pin_configuration_.left_sensor)) {
    dangerousMoveLeft();
  }
  stepper_left_.setCurrentPosition(0);
  while (digitalRead(pin_configuration_.right_sensor)) {
    dangersoursMoveRight();
  }
  stepper_right_.setCurrentPosition(0);
  while (stepper_right_.currentPosition() < MICROSTEPS * STEPS_PER_ROTATION) {
    dangerousMoveLeft();
  }
  is_homed_ = true;
}

void Delta::goTo(float x, float y) {
  // 2 mm per tooth
  // 20 teeth per rotation
  // 355 mm between stops
  float left_mm_position = getLeftTargetPosition(x, y);
  float right_mm_position = getRightTargetPosition(x, y);

  if (isnan(left_mm_position) || isnan(right_mm_position) ||
      (left_mm_position < 0) || (left_mm_position > 355 - 95) ||
      (right_mm_position < 95) || (right_mm_position > 355) ||
      (right_mm_position - left_mm_position > 154) ||
      (right_mm_position - left_mm_position < 94)) {
    Serial.print("Illegal go to: ");
    Serial.print(left_mm_position);
    Serial.print(",");
    Serial.print(right_mm_position);
    Serial.println();
    return;
  }

  // Serial.print(left_mm_position);
  // Serial.print(" ");
  // Serial.print(mmPositionToLeftStepper(left_mm_position));
  // Serial.print(" ");
  // Serial.print(stepper_left_.currentPosition());
  // Serial.print(" ");
  // Serial.print(right_mm_position);
  // Serial.print(" ");
  // Serial.print(mmPositionToRightStepper(right_mm_position));
  // Serial.print(" ");
  // Serial.print(stepper_right_.currentPosition());
  // Serial.println();

  moveTo(mmPositionToLeftStepper(left_mm_position),
         mmPositionToRightStepper(right_mm_position));
}

void Delta::moveTo(int left_position, int right_position) {
  if ((left_position < 0) || (left_position > 265 * 2 * 20) ||
      (right_position < 0) || (right_position > 265 * 2 * 20)) {
    Serial.println("Illegal move to.");
    return;
  }

  bool left_speed_positive = stepper_left_.currentPosition() < left_position;
  bool right_speed_positive = stepper_right_.currentPosition() < right_position;
  int left_speed = left_speed_positive ? SPEED : -SPEED;
  int right_speed = right_speed_positive ? SPEED : -SPEED;

  while (true) {
    if (!digitalRead(pin_configuration_.left_sensor) ||
        !digitalRead(pin_configuration_.right_sensor)) {
      Serial.println("Error, moving triggered sensors.");
      return;
    }

    bool motor_moved = false;
    if (left_speed_positive ? stepper_left_.currentPosition() < left_position
                            : stepper_left_.currentPosition() > left_position) {
      stepper_left_.setSpeed(left_speed);
      stepper_left_.runSpeed();
      motor_moved = true;
    }

    if (right_speed_positive
            ? stepper_right_.currentPosition() < right_position
            : stepper_right_.currentPosition() > right_position) {
      stepper_right_.setSpeed(right_speed);
      stepper_right_.runSpeed();
      motor_moved = true;
    }

    if (!motor_moved) {
      break;
    }
  }
}

float Delta::getRightSliderPosition() {
  return 355 - stepper_right_.currentPosition() * 2 * 20.0 /
                   (MICROSTEPS * STEPS_PER_ROTATION);
}

float Delta::getLeftSliderPosition() {
  return stepper_left_.currentPosition() * 2 * 20.0 /
         (MICROSTEPS * STEPS_PER_ROTATION);
}

int Delta::mmPositionToRightStepper(float mm_position) {
  return (355.0 - mm_position) / (2.0 * 20 / (MICROSTEPS * STEPS_PER_ROTATION));
}

int Delta::mmPositionToLeftStepper(float mm_position) {
  return mm_position / (2.0 * 20 / (MICROSTEPS * STEPS_PER_ROTATION));
}

void Delta::magnetOn() { digitalWrite(pin_configuration_.magnet, HIGH); }

void Delta::magnetOff() { digitalWrite(pin_configuration_.magnet, LOW); }

void Delta::beltForward() {
  digitalWrite(pin_configuration_.belt_forward, HIGH);
  digitalWrite(pin_configuration_.belt_backward, LOW);
}

void Delta::beltBackward() {
  digitalWrite(pin_configuration_.belt_forward, LOW);
  digitalWrite(pin_configuration_.belt_backward, HIGH);
}

void Delta::beltStop() {
  digitalWrite(pin_configuration_.belt_forward, LOW);
  digitalWrite(pin_configuration_.belt_backward, LOW);
}
