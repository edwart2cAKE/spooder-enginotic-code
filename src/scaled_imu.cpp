#include "scaled_imu.hpp"

ScaledIMU::ScaledIMU(uint8_t port, double scalar) : pros::IMU(port), scalar(scalar) {}

double ScaledIMU::get_rotation() const {
    return pros::IMU::get_rotation() * scalar;
}
