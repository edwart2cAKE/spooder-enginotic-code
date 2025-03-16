#include "main.h"
#include "autos.hpp"
#include "lemlib/timer.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

// auton id
int auto_id = 2;
bool auton_ran;

void on_right_button() { auto_id = (auto_id + 1) % num_autos; }

void on_left_button() { auto_id = (auto_id - 1 + num_autos) % num_autos; }

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors

  // bind lcd buttons to auto step functions
  pros::lcd::register_btn0_cb(on_left_button);
  pros::lcd::register_btn2_cb(on_right_button);

  // reverse intake motor
  lady_brown_motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  optical.set_integration_time(100);
  rotation.set_data_rate(5);

  // make intake task
  intake_c.controlIntakeTask();  // start the intake control task
  intake_c.setColorRange(0, 25); // set color range for ring detection

  // print position to brain screen
  auton_ran = false;
  pros::Task screen_task([&]() {
    while (true) {
      // print the auton selector to the brain screen
      pros::lcd::print(5, "Auton: %d", auto_id);
      pros::lcd::print(6, "Auton Name: %s", auto_names[auto_id].c_str());

      // print Lady Brown state to the brain screen
      // pros::lcd::print(7, "Lady Brown State: %d", lift_state);

      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      // print battery wattage to the brain screen
      pros::lcd::print(3, "Wattage: %f",
                       pros::battery::get_voltage() *
                           pros::battery::get_current() / 1e6);

      // print lb pos to brain
     

      // delay to save resources
      pros::delay(20);
    }
  });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  auton_ran = true;
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

  lemlib::Timer auto_timer(60e3); // create a timer for the autonomous
  switch (auto_id) {
  case 0:
    right_red4ring(); // run right red 4 ring auto
    break;
  case 1:
    test_lateral(24); // run test lateral auto
    // test_lateral_range(12, 6, 10);
    break;
  case 2:
    states_skills(); // run states skills auto
    break;
  case 3:
    match_ladder();
    break;
  case 4:
    left_blue2ring();
    break;
  case 5:
    test_angular(90);
    break;
  case 6:
    test_angular_range(45, 45, 5);
    break;
  default:
    pros::lcd::print(5, "You somehow broke it");
    break;
  }
  auto_timer.waitUntilDone(); // wait for the autonomous to finish
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
pros::Controller master(pros::E_CONTROLLER_MASTER);
void opcontrol() {
  mogo_state = LOW;
  mogo.set_value(mogo_state);
  while (true) {
    // tank drive
    chassis.tank(master.get_analog(ANALOG_LEFT_Y),
                 master.get_analog(ANALOG_RIGHT_Y));

    // intake control (R1 / R2)
    int intake_movement = B_INTAKE_UP - B_INTAKE_DOWN;
    intake_c.setDesiredVoltage(intake_movement *
                               127); // set desired voltage for intake

    // mogo control (left)
    if (B_MOGO) {
      mogo_state = !mogo_state;
      mogo.set_value(mogo_state);
    }

    // doinker control
    if (master.get_digital_new_press(DIGITAL_RIGHT)) {
      doinker_state = !doinker_state;
      doinker.set_value(doinker_state);
    }

    // lady brown manual control with acceleration
    control_lift(B_LB_UP - B_LB_DOWN, B_LB_REST, B_LB_READY);

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      autonomous();
      chassis.setBrakeMode(
          pros::E_MOTOR_BRAKE_COAST); // set brake mode to coast after auto
    }
    // delay to save resources

    pros::delay(20);
  }
}