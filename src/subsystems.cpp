#include "subsystems.hpp"
#include "pros/motor_group.hpp"

// port macros

// chassis ports
#define dt_left_ports                                                          \
  { -1, -2, -3 }
#define dt_right_ports                                                         \
  { 5, 4, 6 }

// imu port
#define imu_port 20

// intake port
#define intake_port 7

// lady brown port
#define lady_brown_ports 8

// rotation port
#define rotation_port 11

// mogo port
#define mogo_port 8

// pid constants

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2.5, // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// intake motor
pros::Motor intake(intake_port, pros::v5::MotorGears::blue,
                   pros::v5::MotorEncoderUnits::degrees);

// lady brown motor
pros::MotorGroup lady_brown_motor({lady_brown_ports},
                                  pros::v5::MotorGears::green,
                                  pros::v5::MotorEncoderUnits::degrees);

// mogo piston
pros::adi::DigitalOut mogo(mogo_port, LOW);
bool mogo_state = LOW;

// lift PID controller
lemlib::PID lift_pid(5, 0.0, 0, 5);

// lift exit conditions
lemlib::ExitCondition small_lift_exit_condition(0.5, 100);
lemlib::ExitCondition large_lift_exit_condition(2, 500);

Lift lady_brown(lady_brown_motor, -1, lift_pid, small_lift_exit_condition,
                large_lift_exit_condition);

// chassis motor groups
pros::MotorGroup left_side(dt_left_ports, pros::v5::MotorGears::blue,
                           pros::v5::MotorEncoderUnits::degrees);
pros::MotorGroup right_side(dt_right_ports, pros::v5::MotorGears::blue,
                            pros::v5::MotorEncoderUnits::degrees);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_side,                 // left motor group
                              &right_side,                // right motor group
                              10,                         // 10 inch track width
                              lemlib::Omniwheel::OLD_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2    // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(imu_port);
// odometry settings
lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);
