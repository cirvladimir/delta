#include "input_processor.h"

void Command::print() {
  Serial.println("Command: ");
  if (move_x) {
    Serial.print("X: ");
    Serial.println(x);
  }
  if (move_y) {
    Serial.print("Y: ");
    Serial.println(y);
  }
  if (switch_magnet) {
    if (magnet_on) {
      Serial.println("Magnet on");
    } else {
      Serial.println("Magnet off");
    }
  }
  if (switch_belt) {
    Serial.print("Belt - ");
    Serial.println(belt_state);
  }
}

InputProcessor::InputProcessor()
    : input_state_(InputProcessor::IDLE), read_value_(0),
      command_ready_(false) {}

void InputProcessor::processByte(int input) {
  if (input == -1) {
    return;
  }
  if (input == '\r' || input == '\n' || input == ';') {
    finishNumberInput();
    if (command_.move_x || command_.move_y || command_.switch_magnet ||
        command_.switch_belt) {
      command_ready_ = true;
    }
    input_state_ = IDLE;
    return;
  }
  if (input_state_ == BAD_LINE) {
    return;
  }
  if (('0' <= input) && (input <= '9')) {
    if (input_state_ == READING_X || input_state_ == READING_Y ||
        input_state_ == READING_M || input_state_ == READING_B) {
      read_value_ = read_value_ * 10 + (input - '0');
    } else {
      input_state_ = BAD_LINE;
    }
  } else if (input == 'x' || input == 'X') {
    if (input_state_ != IDLE) {
      input_state_ = BAD_LINE;
    } else {
      read_value_ = 0;
      input_state_ = READING_X;
    }
  } else if (input == 'y' || input == 'Y') {
    if (input_state_ != IDLE) {
      input_state_ = BAD_LINE;
    } else {
      read_value_ = 0;
      input_state_ = READING_Y;
    }
  } else if (input == 'm' || input == 'M') {
    if (input_state_ != IDLE) {
      input_state_ = BAD_LINE;
    } else {
      read_value_ = 0;
      input_state_ = READING_M;
    }
  } else if (input == 'b' || input == 'B') {
    if (input_state_ != IDLE) {
      input_state_ = BAD_LINE;
    } else {
      read_value_ = 0;
      input_state_ = READING_B;
    }
  } else if (input == ' ') {
    finishNumberInput();
    input_state_ = IDLE;
  } else {
    input_state_ = BAD_LINE;
  }

  if (input_state_ == BAD_LINE) {
    command_ = Command();
  }
}

bool InputProcessor::commandReady() { return command_ready_; }

Command InputProcessor::getCommand() {
  command_ready_ = false;
  Command command_copy = command_;
  command_ = Command();
  return command_copy;
}

void InputProcessor::finishNumberInput() {
  if (input_state_ == READING_X) {
    command_.move_x = true;
    command_.x = read_value_;
  }
  if (input_state_ == READING_Y) {
    command_.move_y = true;
    command_.y = read_value_;
  }
  if (input_state_ == READING_M) {
    command_.switch_magnet = true;
    command_.magnet_on = read_value_ > 0;
  }
  if (input_state_ == READING_B) {
    command_.switch_belt = true;
    command_.belt_state = read_value_ == 0   ? Command::STOP
                          : read_value_ == 1 ? Command::FORWARD
                                             : Command::BACKWARD;
  }
}