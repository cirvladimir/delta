#include <Arduino.h>
#include <SoftwareSerial.h>

#include "delta.h"
#include "input_processor.h"

PinConfiguration pin_config = {.left_motor_step = 3,
                               .left_motor_dir = 2,
                               .left_sensor = 8,
                               .right_motor_step = 5,
                               .right_motor_dir = 4,
                               .right_sensor = 7,
                               .magnet = 6,
                               .belt_forward = 9,
                               .belt_backward = 10};

Delta delta(pin_config);

InputProcessor input_processor;
float x = 0;
float y = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(999999);

  delta.setup();
  delta.home();
  Serial.println("ready");
}

void loop() {
  input_processor.processByte(Serial.read());
  if (input_processor.commandReady()) {
    Command command = input_processor.getCommand();
    command.print();
    if (command.switch_magnet) {
      if (command.magnet_on) {
        delta.magnetOn();
      } else {
        delta.magnetOff();
      }
    }
    if (command.move_x || command.move_y) {
      x = command.move_x ? command.x : x;
      y = command.move_y ? command.y : y;
      delta.goTo(x, y);
    }
    if (command.switch_belt) {
      switch (command.belt_state) {
      case Command::STOP:
        delta.beltStop();
        break;
      case Command::FORWARD:
        delta.beltForward();
        break;
      case Command::BACKWARD:
        delta.beltBackward();
        break;
      }
    }
    Serial.println("ready");
  }
}
