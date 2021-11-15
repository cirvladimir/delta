#include "geometry.h"

float getLeftTargetPosition(float x, float y) {
  float pivot_y = (93.0 + 10.0) - (93.0 - 44.263);
  float left_radius = 54.263;
  float pivot_x =
      x - sqrt(left_radius * left_radius - (pivot_y - y) * (pivot_y - y));
  float target_x = pivot_x + 3.0 + 1.4 - 11.0;
  return target_x;
}

float getRightTargetPosition(float x, float y) {
  float pivot_y = (93.0 - 6.288) - (93.0 - 48.75);
  float left_radius = 50.0;
  float pivot_x =
      x + sqrt(left_radius * left_radius - (pivot_y - y) * (pivot_y - y));
  float target_x = pivot_x + 22.0 + 40.573 - 1.0;
  return target_x;
}