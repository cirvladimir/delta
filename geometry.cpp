#include "geometry.h"
// #include <cmath>
// #include <iostream>
// using std::isnan;

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

bool positionLegal(float x, float y) {
  float left_mm_position = getLeftTargetPosition(x, y);
  float right_mm_position = getRightTargetPosition(x, y);

  if (isnan(left_mm_position) || isnan(right_mm_position) ||
      (left_mm_position < 0) || (left_mm_position > 355 - 95) ||
      (right_mm_position < 95) || (right_mm_position > 355) ||
      (right_mm_position - left_mm_position > 154) ||
      (right_mm_position - left_mm_position < 94)) {
    return false;
  }
  return true;
}

// int main() {
//   float x = 1000.0;
//   while (!positionLegal(x, 15.0)) {
//     x -= 1.0;
//   }
//   std::cout << x << std::endl;
//   return 0;
// }
