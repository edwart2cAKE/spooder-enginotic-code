#include "subsystems.hpp"
#include "lemlib/timer.hpp"
#include "pros/llemu.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"

// port macros

// chassis ports
#define dt_left_ports {-1, -2, -3}
#define dt_right_ports {5, 4, 6}

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

// lb Constants TODO: Use Enums post states
#define REST 0
#define READY 1
#define DRIVER 2

// pid constants

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(8,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       0,   // derivative gain (kD)
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
                       10,  // derivative gain (kD)
                       10,  // anti windup
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
lemlib::PID lift_pid(3, 0, 5, 5);
int lift_state = 0;

// optical sensor
pros::Optical optical(9);

double lift_error(int state) {
  int target = (state == READY) ? -93 : -1;
  const double error = target - lady_brown_motor.get_position(0);
  return error;
}

// lift controller function
void control_lift(int up_down, bool rest, bool ready) {
  // state changes

  // ready button -> READY
  if (ready) {
    lift_state = READY;
  }

  // rest button -> REST
  if (rest) {
    lift_state = REST;
  }

  // up_down + READY -> DRIVER
  if (up_down != 0 && lift_state == READY) {
    lift_state = DRIVER;
  }

  // pid control
  if (lift_state == READY || lift_state == REST) {

    const float output = lift_pid.update(lift_error(lift_state));
    lady_brown_motor.move(output);
  }

  // driver control
  if (lift_state == DRIVER) {
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
  // jam detection based on
  static bool jammed = false;
  static lemlib::Timer jam_timer(500);
  static lemlib::Timer anti_jam_timer(1000);

  static lemlib::Timer color_sort_timer(10); // timer for color sorting
  static lemlib::Timer color_sort_stop_timer(
      200); // timer for stopping color sort
  static bool was_blue = false;
  bool is_blue = 250 > optical.get_hue() &&
                 optical.get_hue() > 200; // check if color is blue

  if (intake.get_actual_velocity() > -10 && intake.get_actual_velocity() < 10 &&
      intake.get_current_draw() > 0.7 * intake.get_current_limit()) {
    jam_timer.resume();
    if (jam_timer.isDone()) {
      jammed = true;          // jam detected
      anti_jam_timer.reset(); // reset anti jam timer
    }
  } else {
    jam_timer.pause();
    jam_timer.reset();
    jammed = false; // no jam detected
  }

  // intake control
  if (!jammed) {
    if (up_down > 0) {
      intake.move(127); // move intake up
    } else if (up_down < 0) {
      intake.move(-127); // move intake down
    } else {
      intake.move(0); // stop intake
    }
    if (was_blue && !is_blue) {
      color_sort_timer.resume(); // start color sort timer at falling edge
    }
    if (color_sort_timer.isDone() && !color_sort_stop_timer.isDone()) {
      intake.move(0);                 // stop intake when color is detected
      color_sort_stop_timer.resume(); // start timer to stop color sort
    }
    if (color_sort_stop_timer.isDone()) {
      color_sort_timer.pause(); // pause color sort timer
      color_sort_timer.reset(); // reset stop timer
      color_sort_stop_timer.reset(); // reset color sort timer
      color_sort_stop_timer.pause(); // pause stop timer
    }
  } else
  // jammed intake control
  {
    intake.move(-127); // stop intake if jammed
    // start anti jam timer
    anti_jam_timer.resume();
  }
  was_blue = is_blue; // store previous color state
  pros::lcd::print(7, "Color Sort Timer %d", color_sort_timer.getTimeLeft());
}

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