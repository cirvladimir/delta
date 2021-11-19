#pragma once

#include <Arduino.h>

struct Command {
  float x = 0, y = 0;
  bool magnet_on = false;
  enum BeltState { STOP, FORWARD, BACKWARD };
  BeltState belt_state = STOP;

  bool move_x = false, move_y = false, switch_magnet = false,
       switch_belt = false;

  void print();
};

class InputProcessor {
public:
  InputProcessor();
  void processByte(int input);
  bool commandReady();
  Command getCommand();

private:
  void finishNumberInput();

  enum InputState {
    IDLE,
    READING_X,
    READING_Y,
    READING_M,
    READING_B,
    BAD_LINE
  };
  InputState input_state_;

  float read_value_;

  bool command_ready_;
  Command command_;
};