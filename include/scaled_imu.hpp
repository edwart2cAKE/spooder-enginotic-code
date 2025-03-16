#pragma once

#include "pros/imu.hpp"

class ScaledIMU {
public:
  ScaledIMU(uint8_t port, double scalar);

  double get_rotation() const;

private:
  pros::Imu imu;
  double scalar;
};
