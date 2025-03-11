#ifndef SUBSYSTEMS_HPP
#define SUBSYSTEMS_HPP
// IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"       // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp" // IWYU pragma: keep
#include "lemlib/exitcondition.hpp"         // IWYU pragma: keep
#include "lemlib/pid.hpp"                   // IWYU pragma: keep
#include "lift.hpp"
#include "pros/abstract_motor.hpp"          // IWYU pragma: keep
#include "pros/adi.h"                       // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

// Intake motor
extern pros::Motor intake;

// Lady Brown motor
extern Lift lady_brown;
extern pros::MotorGroup lady_brown_motor;

// mogo
extern pros::adi::DigitalOut mogo;
extern bool mogo_state;

// Direct Chassis object from lemlib
extern lemlib::Chassis chassis;

#endif // SUBSYSTEMS_HPP