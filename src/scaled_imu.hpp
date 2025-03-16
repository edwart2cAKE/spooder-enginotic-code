#ifndef SCALED_IMU_HPP
#define SCALED_IMU_HPP

#include "pros/imu.hpp"

class ScaledIMU : public pros::IMU {
public:
    ScaledIMU(uint8_t port, double scalar);
    double get_rotation() const;

private:
    double scalar;
};

#endif // SCALED_IMU_HPP
