#include "subsystems.hpp"
#include "intake.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "scaled_imu.hpp"

// port macros

// chassis ports
#define dt_left_ports {-1, -2, -3}
#define dt_right_ports {5, 4, 6}

// imu port
#define imu_port 19

// intake port
#define intake_port 7

// lady brown port
#define lady_brown_ports 8

// rotation port
#define rotation_port 9

// mogo port
#define mogo_port 8

// lb Constants TODO: Use Enums post states
#define REST 0
#define READY 1
#define DRIVER 2

// pid constants

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(9,   // proportional gain (kP)
                       0.1, // integral gain (kI)
                       10,  // derivative gain (kD)
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
                       0.1, // integral gain (kI)
                       20,  // derivative gain (kD)
                       10,  // anti windup
                       3,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       5,   // large error range, in degrees
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

// doinket

pros::adi::DigitalOut doinker(7, LOW);
bool doinker_state = LOW;

// lift PID controller
lemlib::PID lift_pid(3, 0, 5, 10);
int lift_state = 0;

// optical sensor
pros::Optical optical(10);

// rotation sensor
pros::Rotation rotation(rotation_port);

// create Intake object
Intake intake_c(intake, optical);

double lift_error(int state) {
  if (!pros::lcd::is_initialized()) {
    pros::lcd::initialize();
  } // initialize lcd if not already done

  static int rotations;
  static int prev_rotation = 0;
  int angle;

  int target = (state == REST) ? 359 : 245-3; // set target

  if (rotation.get_position() < 10000) {
    angle = lady_brown_motor.get_position();
  } else {
    angle = rotation.get_position()/100; 
  }

  pros::lcd::print(7, "Angle: %d", angle); // print angle to lcd

  const double error = target - angle;     // calculate error
  prev_rotation = rotation.get_position(); // store previous rotation
  return error;
}

// lift controller function
void control_lift(int up_down, bool rest, bool ready) {
  // state changes

  // ready button -> READY
  if (ready) {
    lift_state = READY;
    intake_c.setAntiJamming(false); // disable anti-jamming when ready
  }

  // rest button -> REST
  if (rest) {
    lift_state = REST;
    intake_c.setAntiJamming(true); // enable anti-jamming when resting
  }

  // up_down + READY -> DRIVER
  if (up_down != 0) {
    lift_state = DRIVER;
  }

  // pid control
  if (lift_state == READY || lift_state == REST) {

    const float output = lift_pid.update(lift_error(lift_state));

    if (lift_state == REST && lift_error(lift_state) < 1) {
      lady_brown_motor.brake(); // brake when at rest and above target
    } else
      lady_brown_motor.move(output);
  }

  // driver control
  if (lift_state == DRIVER) {
    int _ = lift_error(lift_state); // get lift error for driver control
    intake_c.setAntiJamming(false); // disable anti-jamming when driving
    static int lady_brown_speed = 0;
    if (up_down != 0) {
      lady_brown_speed += (up_down * 10);
      lady_brown_motor.move(lady_brown_speed);
    } else {
      lady_brown_motor.brake();
      lady_brown_speed = 0;
    }
  }
}

// intake control with anti jam
void intake_control(int up_down) {
  // intake_c.controlIntake(up_down);
}

// chassis motor groups
pros::MotorGroup left_side(dt_left_ports, pros::v5::MotorGears::blue,
                           pros::v5::MotorEncoderUnits::degrees);
pros::MotorGroup right_side(dt_right_ports, pros::v5::MotorGears::blue,
                            pros::v5::MotorEncoderUnits::degrees);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_side,  // left motor group
                              &right_side, // right motor group
                              10,          // 10 inch track width
                              lemlib::Omniwheel::OLD_275 -
                                  0.06, // using new 4" omnis
                              450,      // drivetrain rpm is 360
                              2         // horizontal drift is 2 (for now)
);

// imu
ScaledIMU imu(imu_port, (360.0 * 7 / (360 * 7 - 12)));
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