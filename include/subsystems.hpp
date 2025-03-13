#ifndef SUBSYSTEMS_HPP
#define SUBSYSTEMS_HPP
// IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"       // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp" // IWYU pragma: keep
#include "lemlib/exitcondition.hpp"         // IWYU pragma: keep
#include "lemlib/pid.hpp"                   // IWYU pragma: keep
#include "pros/abstract_motor.hpp"          // IWYU pragma: keep
#include "pros/adi.h"                       // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

// Intake motor
extern pros::Motor intake;

// Lady Brown motor
extern pros::MotorGroup lady_brown_motor;
extern void control_lift(int up_down, bool rest, bool ready);
extern double lift_error(int lift_state);
extern int lift_state;

// mogo
extern pros::adi::DigitalOut mogo;
extern bool mogo_state;

// Direct Chassis object from lemlib
extern lemlib::Chassis chassis;

// Controls
#define B_MOGO master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)

#define B_INTAKE_UP master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)
#define B_INTAKE_DOWN master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)

#define B_LB_UP master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)
#define B_LB_DOWN master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)
#define B_LB_REST master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)
#define B_LB_READY master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)

#endif // SUBSYSTEMS_HPP