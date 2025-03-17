#include "autos.hpp"
#include "intake.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include <cmath>
#include <cstdio>
#include <sys/_intsup.h>

#define LB_TIMEOUT 1000 // timeout for lady brown to finish moving

void test_lateral(int dist = 12) {
  chassis.setPose(0, 0, chassis.getPose().theta);
  int start_time = pros::millis();
  chassis.moveToPoint(0, dist, std::max(500, abs(dist) * 300),
                      {.forwards = (dist > 0)});
  chassis.waitUntilDone();
  int end_time = pros::millis();
  printf("Time: %d\n", end_time - start_time);
  printf("Distance: %d\n", dist);
}

void test_lateral_range(int start_dist = 12, int dist_increment = 6,
                        int num_tests = 5) {
  // without using test_lateral as it resets the pose
  chassis.setPose(0, 0, 0);
  for (int i = 0; i < num_tests; i++) {
    int length = start_dist + i * dist_increment;
    test_lateral(length * (i % 2 == 0 ? 1 : -1));
    pros::delay(1500);
  }
}

void test_angular(int a = 90) {
  int start_time = pros::millis();
  float start_theta = chassis.getPose().theta;
  chassis.turnToHeading(chassis.getPose().theta + a, 1e4);
  chassis.waitUntilDone();
  int end_time = pros::millis();
  float end_theta = chassis.getPose().theta;
  printf("Time: %d\n", end_time - start_time);
  printf("Angle: %d\n", a);
  printf("Error: %f\n", end_theta - start_theta - a);
}
void test_angular_range(int start_angle = 45, int angle_increment = 45,
                        int num_tests = 5) {
  chassis.setPose(0, 0, 0);
  for (int i = 0; i < num_tests; i++) {
    int length = start_angle + i * angle_increment;
    test_angular(length);
  }
}

// auto skills driver
void auto_skills_driver() {
  // set position
  chassis.setPose(0, -61, 0);

  // score ring in intake
  intake.move(127);
  pros::delay(500);
  intake.move(0);
  chassis.tank(0, 0);
  intake.move(-127);

  // get right mogo
  chassis.moveToPoint(0, -46.75, 1750);
  pros::delay(500);
  intake.move(0);
  chassis.turnToHeading(-90, 1750);
  chassis.moveToPoint(20, -49.5, 3500, {.forwards = false, .maxSpeed = 50});
  chassis.waitUntilDone();
  mogo.set_value(1);
  pros::delay(500);
}

// auto skills
void auto_skills() {
  // set position
  auto_skills_driver();
  // get group of 5 rings
  chassis.turnToPoint(24, -24, 1500, {.maxSpeed = 90});
  chassis.waitUntilDone();
  intake.move(127);
  chassis.moveToPoint(24, -24, 2000, {.maxSpeed = 90});
  pros::delay(1000);
  chassis.turnToHeading(90, 1500);
  chassis.moveToPoint(48, -24, 1500, {.maxSpeed = 90});
  chassis.waitUntilDone();
  chassis.turnToHeading(180, 1500);
  chassis.moveToPoint(48, -65, 3000, {.maxSpeed = 55});
  chassis.waitUntilDone();

  chassis.moveToPoint(48, -48, 1500, {.forwards = false, .maxSpeed = 90});
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1500);
  chassis.moveToPoint(60, -48, 1500, {.maxSpeed = 45, .minSpeed = 70});
  chassis.waitUntilDone();
  pros::delay(1000);
  chassis.moveToPoint(48, -48, 1200, {.forwards = false, .maxSpeed = 90});

  // score rings in goal
  chassis.turnToHeading(-45, 1200);
  chassis.moveToPoint(60, -60, 2000, {.forwards = false, .maxSpeed = 90});
  chassis.waitUntilDone();
  mogo.set_value(0);
  intake.move(0);

  // chassis.setPose(57, -57, -45);
  // move to other side
  // copy of above code but mirrored
  // get left mogo
  chassis.moveToPoint(48, -48, 1500, {.maxSpeed = 50});
  chassis.waitUntilDone();
  chassis.turnToHeading(-90, 1500);
  chassis.moveToPoint(72, -48, 3500, {.forwards = false, .maxSpeed = 90});
  chassis.waitUntilDone();
  chassis.setPose(64, -48, chassis.getPose().theta);
  chassis.moveToPoint(48, -48, 1500, {.maxSpeed = 50});
  chassis.waitUntilDone();
  chassis.turnToHeading(90, 1500);
  pros::delay(1000);

  chassis.moveToPoint(-24, -49.5, 6000, {.forwards = false, .maxSpeed = 55});
  chassis.waitUntilDone();
  mogo.set_value(1);

  // get group of 5 rings
  chassis.turnToPoint(-24, -24, 1500, {.maxSpeed = 90});
  chassis.waitUntilDone();
  intake.move(127);
  chassis.moveToPoint(-24, -24, 2000, {.maxSpeed = 90});
  pros::delay(1000);
  chassis.turnToHeading(-90, 1500);
  chassis.moveToPoint(-48, -24, 1500, {.maxSpeed = 90});
  chassis.waitUntilDone();
  chassis.turnToHeading(180, 1500);
  chassis.moveToPoint(-48, -65, 3000, {.maxSpeed = 55});
  chassis.waitUntilDone();

  chassis.moveToPoint(-48, -48, 1500, {.forwards = false, .maxSpeed = 90});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1500);
  chassis.moveToPoint(-60, -48, 1500, {.maxSpeed = 45, .minSpeed = 70});
  chassis.waitUntilDone();
  pros::delay(1000);
  chassis.moveToPoint(-48, -48, 1200, {.forwards = false, .maxSpeed = 90});

  // score rings in goal
  chassis.turnToHeading(45, 1200);
  chassis.moveToPoint(-60, -60, 2000, {.forwards = false, .maxSpeed = 90});
  chassis.waitUntilDone();
  mogo.set_value(0);
  intake.move(0);
  chassis.moveToPoint(-48, -48, 1500);

  // go to other side
  chassis.turnToPoint(-48, 0, 1500);
  chassis.moveToPoint(-48, 0, 1500);

  chassis.turnToPoint(0, 60, 1500);
  chassis.moveToPoint(0, 60, 1500);

  // push first mogo
  chassis.turnToPoint(-60, 60, 1500, {.forwards = false});
  chassis.moveToPoint(-60, 60, 1500, {.forwards = false});

  // repush
  chassis.moveToPoint(-40, 60, 1200);
  chassis.tank(-90, -90);
  pros::delay(700);
  chassis.tank(0, 0);

  // 2nd mogo push
  chassis.moveToPoint(60, 60, 1500);
  chassis.moveToPoint(40, 60, 1200, {.forwards = false});
  chassis.tank(90, 90);
  pros::delay(700);
  chassis.tank(0, 0);

  //*/
}

void new_auto_skills_push() {
  auto_skills_driver();
  // push first mogo
  chassis.turnToPoint(56, -56, 1200, {.forwards = false});
  chassis.moveToPoint(56, -56, 2000, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(0);
  pros::delay(500);

  // go to second
  chassis.moveToPoint(20, -49.5, 1200);
  chassis.turnToPoint(-20, -49.5, 1500, {.forwards = false});
  chassis.moveToPoint(20, -49.5, 1500, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(1);
  pros::delay(500);

  // push 2nd
  chassis.turnToPoint(-56, -56, 1200, {.forwards = false});
  chassis.moveToPoint(-56, -56, 2000, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(0);
  pros::delay(500);

  // go to 3rd
  chassis.moveToPoint(-20, -49.5, 1200);
  chassis.turnToPoint(-48, 0, 1500);
  chassis.moveToPoint(-48, 0, 2000);
  chassis.turnToPoint(-24, 60, 1200, {.forwards = false});
  chassis.moveToPoint(-24, 60, 2300);
  chassis.waitUntilDone();
  mogo.set_value(1);
  pros::delay(500);

  // push 3rd mogo
  chassis.turnToPoint(-56, 56, 1200, {.forwards = false});
  chassis.moveToPoint(-56, 56, 2000, {.forwards = false});
  chassis.waitUntilDone();
  pros::delay(500);

  // go to 4th
  chassis.moveToPoint(-20, 60, 1200);
  chassis.turnToPoint(-20, 60, 1500, {.forwards = false});
  chassis.moveToPoint(20, 60, 1500, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(1);
  pros::delay(500);

  // score 4th
  chassis.turnToPoint(56, 56, 1200, {.forwards = false});
  chassis.moveToPoint(56, 56, 2000, {.forwards = false});
  chassis.waitUntilDone();
  pros::delay(500);
}

void skills_2_push() {
  chassis.setPose(-48, -48, 0);

  // move to middle
  chassis.moveToPoint(-48, 0, 1200, {.minSpeed = 100, .earlyExitRange = 4});
  chassis.moveToPoint(-8, 60, 2500);

  // get first mogo
  chassis.turnToHeading(90, 1200);
  chassis.moveToPoint(-20, 60, 2200, {.forwards = false, .maxSpeed = 80});
  chassis.waitUntilDone();
  mogo.set_value(1);
  pros::delay(500);

  // push first mogo
  chassis.moveToPoint(-60, 60, 1500, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(0);
  pros::delay(500);

  // go to second
  chassis.moveToPoint(-8, 60, 2500);
  chassis.turnToHeading(-90, 1200);
  chassis.moveToPoint(20, 60, 2200, {.forwards = false, .maxSpeed = 80});
  chassis.waitUntilDone();
  mogo.set_value(1);
  pros::delay(500);

  // push 2nd mogo
  chassis.moveToPoint(60, 60, 1500, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(0);
  pros::delay(500);
}
// match ladder
void match_ladder() {
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 30, 2000);
  chassis.turnToHeading(45, 4000);
  chassis.waitUntilDone();
  chassis.moveToPoint(5, 35, 1200);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lady_brown_motor.move_relative(-540, 100);
}

void match_ring2() {
  chassis.setPose(-36, -60, 180);
  chassis.moveToPoint(-36, -24 - 12 * std::sqrt(3), 1200, {.forwards = false});
  chassis.turnToPoint(-24, -24, 1200, {.forwards = false});
  chassis.moveToPoint(-24, -24, 1200, {.forwards = false});

  chassis.waitUntilDone();
  mogo.set_value(1);
  pros::delay(500);
  intake.move(127);
  chassis.turnToHeading(-90, 1200);
  chassis.moveToPoint(-60, -24, 1500);
  chassis.waitUntilDone();
  pros::delay(300);
  chassis.tank(-30, -30);
  pros::delay(300);
  chassis.tank(0, 0);
}

void right_red4ring() {
  chassis.setPose(14.5, -60.5, 230);        // set position
  lady_brown_motor.set_zero_position(-100); // reset lb position
  intake_c.setColorRange(200, 250);         // throw away blue rings

  // score on alliance stake
  chassis.tank(35, 35); // move forward to score on alliance stake
  lady_brown_motor.move_relative(
      -600, 200);     // move lb down to score on alliance stake
  pros::delay(300);   // wait for 300ms
  chassis.tank(0, 0); // stop moving
                      // move lb down to score on alliance stake
  int timeout = 0;
  while (lady_brown_motor.get_position(0) > -600 &&
         timeout < 500) { // wait for lb to finish moving
    pros::delay(10);
    timeout += 10; // increment timeout
  }
  chassis.tank(-90, -90); // move back to starting position
  pros::delay(300);       // wait for 300ms
  lady_brown_motor.move_relative(700,
                                 200); // move lb back up to starting position
  chassis.tank(0, 0);                  // stop moving
  intake_c.setDesiredVoltage(127);     // start intake to get rings

  // go to mogo
  chassis.turnToPoint(25, -48, 900, {.forwards = false});
  chassis.moveToPoint(25, -48, 1200, {.forwards = false});

  chassis.turnToHeading(180, 1200); // turn to mogo
  chassis.moveToPoint(25, -23, 900,
                      {.forwards = false, .maxSpeed = 60}); // move to mogo
  chassis.waitUntilDone(); // wait for move to finish
  mogo.set_value(1);       // get mogo
  pros::delay(300);        // wait for mogo to get up
  chassis.turnToHeading(90, 900);
  chassis.moveToPoint(48, -22, 1200); // move to mogo stack
  chassis.waitUntilDone();            // wait for move to finish
  pros::delay(500);

  // go to ladder
  chassis.turnToHeading(-90, 1200);
  chassis.moveToPoint(24, -24, 1200); // move to ladder
  chassis.turnToHeading(-45, 1200);
  // chassis.moveToPoint(16, -16, 1200); // move to ladder
  lady_brown_motor.move_relative(-540, 100); // move lb down to score on ladder

  /*/ go to middle 2 ring stack
  chassis.turnToPoint(24, -48, 1200);
  chassis.moveToPoint(24, -48, 1200);

  chassis.turnToHeading(-90, 1200);
  chassis.moveToPoint(2, -48, 3000,
                      {.maxSpeed = 70}); // move to middle 2 ring stack
  chassis.waitUntilDone(); // wait for move to finish
  pros::delay(300);
  chassis.moveToPoint(6, -48, 1200, {.forwards = false}); // move to middle 2
  ring stack chassis.moveToPoint(-6, -48, 1200); chassis.waitUntilDone(); //
  wait for move to finish


  // go to ladder
  chassis.turnToHeading(-90, 1200);
  chassis.moveToPoint(24, -24, 1200); // move to ladder
  chassis.turnToHeading(-45, 1200);
  chassis.moveToPoint(16, -16, 1200);
  lady_brown_motor.move_relative(-540, 100); // move lb down to score on ladder

  //comment trap */
  chassis.waitUntilDone(); // wait for move to finish
}

void left_blue2ring() {
  chassis.setPose(-14.5, -60.5, -230);      // set position
  lady_brown_motor.set_zero_position(-100); // reset lb position
  intake_c.setColorRange(0, 50);            // throw away red rings

  // score on alliance stake
  chassis.tank(35, 35); // move forward to score on alliance stake
  lady_brown_motor.move_relative(
      -600, 200);     // move lb down to score on alliance stake
  pros::delay(300);   // wait for 300ms
  chassis.tank(0, 0); // stop moving
                      // move lb down to score on alliance stake
  int timeout = 0;
  while (lady_brown_motor.get_position(0) > -600 &&
         timeout < 500) { // wait for lb to finish moving
    pros::delay(10);
    timeout += 10; // increment timeout
  }
  chassis.tank(-90, -90); // move back to starting position
  pros::delay(300);       // wait for 300ms
  lady_brown_motor.move_relative(700,
                                 200); // move lb back up to starting position
  chassis.tank(0, 0);                  // stop moving
  intake_c.setDesiredVoltage(127);     // start intake to get rings

  // go to mogo
  chassis.turnToPoint(-24, -48, 900, {.forwards = false});
  chassis.moveToPoint(-24, -48, 1200, {.forwards = false});

  chassis.turnToHeading(180, 1200); // turn to mogo
  chassis.moveToPoint(-24, -26, 900,
                      {.forwards = false, .maxSpeed = 60}); // move to mogo
  chassis.waitUntilDone(); // wait for move to finish
  mogo.set_value(1);       // get mogo
  pros::delay(300);        // wait for mogo to get up
  chassis.turnToHeading(-90, 900);
  chassis.moveToPoint(-48, -26, 1200); // move to mogo stack
  chassis.waitUntilDone();             // wait for move to finish
  pros::delay(500);

  // go to ladder
  chassis.turnToHeading(-90, 1200);
  chassis.moveToPoint(-24, -24, 1200); // move to ladder
  chassis.turnToHeading(45, 1200);
  // chassis.moveToPoint(16, -16, 1200); // move to ladder
  lady_brown_motor.move_relative(-540, 100); // move lb down to score on ladder
  chassis.waitUntilDone();
  chassis.tank(20, 20);

  /*/ go to middle 2 ring stack
  chassis.turnToPoint(24, -48, 1200);
  chassis.moveToPoint(24, -48, 1200);

  chassis.turnToHeading(-90, 1200);
  chassis.moveToPoint(2, -48, 3000,
                      {.maxSpeed = 70}); // move to middle 2 ring stack
  chassis.waitUntilDone(); // wait for move to finish
  pros::delay(300);
  chassis.moveToPoint(6, -48, 1200, {.forwards = false}); // move to middle 2
  ring stack chassis.moveToPoint(-6, -48, 1200); chassis.waitUntilDone(); //
  wait for move to finish


  // go to ladder
  chassis.turnToHeading(-90, 1200);
  chassis.moveToPoint(24, -24, 1200); // move to ladder
  chassis.turnToHeading(-45, 1200);
  chassis.moveToPoint(16, -16, 1200);
  lady_brown_motor.move_relative(-540, 100); // move lb down to score on ladder

  //comment trap */
  chassis.waitUntilDone(); // wait for move to finish
}

#define CHAIN {.maxSpeed = 110, .minSpeed = 55, .earlyExitRange = 10}
void states_skills() {
  // chain for moves

  chassis.setPose(0, -56, 180);             // set position
  lady_brown_motor.set_zero_position(-100); // reset lb position
  intake_c.setColorRange(200, 250);         // throw away blue rings

  // score on alliance stake
  lady_brown_motor.move_absolute(
      -700, 200); // move lb down to score on alliance stake
  int timeout = 0;
  while (lady_brown_motor.get_position(0) > -700 &&
         timeout < LB_TIMEOUT) { // wait for lb to finish moving
    pros::delay(10);
    timeout += 10; // increment timeout
  }

  // move lb back up to starting position
  intake_c.setDesiredVoltage(127); // start intake to get rings

  // get first mogo
  chassis.moveToPoint(0, -48, 1200, {.forwards = false}); // move to mogo
  chassis.waitUntil(10); // wait for move to finish
  lady_brown_motor.move_absolute(0, 200);
  chassis.turnToHeading(-90, 1200); // turn to mogo
  chassis.moveToPoint(24, -48, 1200,
                      {.forwards = false, .maxSpeed = 80}); // move to mogo
  chassis.waitUntilDone(); // wait for move to finish
  mogo.set_value(1);       // get mogo
  pros::delay(300);        // wait for mogo to get up

  // get first ring
  chassis.turnToPoint(23, -24, 1200);
  chassis.moveToPoint(23, -24, 1200,
                      CHAIN); // move to ring

  // go to 2nd ring
  chassis.turnToPoint(37, 0, 1200, CHAIN);
  chassis.moveToPoint(37, 0, 1200, CHAIN); // move to 2nd ring

  chassis.turnToPoint(48, 24, 1200, CHAIN);
  chassis.moveToPoint(48, 24, 1200); // move to 2nd ring

  // go back
  chassis.turnToPoint(40, -1, 1200, {.forwards = false});
  chassis.moveToPoint(40, -1, 1200, {.forwards = false});

  // comment trap */

  // score wall stake
  // stop intake
  chassis.turnToHeading(90, 1200); // turn to mogo stack
  intake_c.setAntiJamming(false);  // turn off anti-jamming
  lady_brown_motor.move_absolute(-115, 200);
  chassis.moveToPoint(70, -1, 1800, {.maxSpeed = 50});
  chassis.waitUntil(12);
  pros::delay(1000); // wait for 1s to get rings in
  intake_c.setDesiredVoltage(-127);
  pros::delay(50);                 // wait for 300ms to get rings in
  intake_c.setDesiredVoltage(127); // stop intake

  chassis.waitUntilDone(); // wait for move to finish
  chassis.tank(30, 30);
  pros::delay(50);
  intake_c.setDesiredVoltage(-127); // stop intake
  pros::delay(20);
  intake_c.setDesiredVoltage(127);
  pros::delay(50);
  intake_c.setDesiredVoltage(-127); // stop intake
  pros::delay(20);
  intake_c.setDesiredVoltage(127);
  pros::delay(50);

  intake_c.setDesiredVoltage(0); // stop intake
  lady_brown_motor.move_absolute(-500,
                                 200); // move lb down to score on wall stake
  timeout = 0;
  pros::delay(300); // wait for 300ms to get rings in
  while (lady_brown_motor.get_position(0) > -500 &&
         timeout < LB_TIMEOUT) { // wait for lb to finish moving
    pros::delay(10);
    timeout += 10; // increment timeout
  }

  // go to ring strip
  intake_c.setAntiJamming(true);
  intake_c.setDesiredVoltage(127); // start intake to get rings
  chassis.tank(-90, -90);          // move back to starting position
  pros::delay(300);                // wait for 300ms
  chassis.tank(0, 0);              // stop moving

  chassis.moveToPoint(48, 0, 1200, {.forwards = false}); // move to mogo stack
  chassis.turnToHeading(180, 1200);                      // turn to mogo stack
  chassis.moveToPoint(48, -48, 1500);                    // move to mogo stack
  chassis.moveToPoint(48, -65, 2000, {.maxSpeed = 50});  // move to mogo stack
  chassis.waitUntilDone(); // wait for move to finish

  // get last ring
  chassis.turnToPoint(60, -48, 1200);
  chassis.moveToPoint(60, -48, 700); // move to last ring
  chassis.waitUntilDone();           // wait for move to finish
  pros::delay(300);                  // wait for ring to get in

  // score mogo in corner
  chassis.turnToPoint(66, -66, 1200, {.forwards = false}); // turn to corner
  chassis.waitUntilDone();
  chassis.moveToPoint(66, -66, 1200, {.forwards = false}); // move to corner
  chassis.waitUntilDone();
  mogo.set_value(0);

  // prep for 2nd mogo
  chassis.turnToPoint(-2, -48, 1200);
  lady_brown_motor.move_absolute(0, 200); // move lb up to starting position
  chassis.moveToPoint(-2, -48, 2200, {.maxSpeed = 90});

  // move to second mogo
  chassis.turnToPoint(-24, -48, 1200, {.forwards = false});
  chassis.moveToPoint(
      -24, -48, 1200,
      {.forwards = false, .maxSpeed = 70}); // move to second mogo
  chassis.waitUntilDone();                  // wait for move to finish
  mogo.set_value(1);                        // get second mogo
  pros::delay(300);                         // wait for mogo to get up
  chassis.setPose(-20, chassis.getPose().y,
                  chassis.getPose().theta); // set pose for next move

  // get first ring
  chassis.turnToPoint(-24, -24, 1200);
  chassis.moveToPoint(-24, -24, 1200,
                      CHAIN); // move to ring

  // get 2nd ring
  chassis.turnToPoint(-43, 0, 1200, CHAIN);
  chassis.moveToPoint(-43, 0, 1200, CHAIN); // move to 2nd ring
  chassis.turnToPoint(-48, 24, 1200, CHAIN);
  chassis.moveToPoint(-48, 24, 1200); // move to 2nd ring

  // go back
  chassis.turnToPoint(-40, -3, 1200, {.forwards = false});
  chassis.moveToPoint(-40, -3, 1200, {.forwards = false});

  // comment trap */

  // score wall stake
  intake_c.setDesiredVoltage(127);  // turn off intake to avoid jamming
                                    // stop intake
  chassis.turnToHeading(-90, 1200); // turn to mogo stack
  intake_c.setAntiJamming(false);   // turn off anti-jamming
  lady_brown_motor.move_absolute(-115, 200);
  chassis.moveToPoint(-70, -3, 1800, {.maxSpeed = 50});
  chassis.waitUntil(12);
  pros::delay(1000); // wait for 1s to get rings in
  intake_c.setDesiredVoltage(-127);
  pros::delay(20);                 // wait for 300ms to get rings in
  intake_c.setDesiredVoltage(127); // stop intake

  chassis.waitUntilDone(); // wait for move to finish
  chassis.tank(30, 30);
  pros::delay(50);
  intake_c.setDesiredVoltage(-127); // stop intake
  pros::delay(20);
  intake_c.setDesiredVoltage(127);
  pros::delay(50);
  intake_c.setDesiredVoltage(-127); // stop intake
  pros::delay(20);
  intake_c.setDesiredVoltage(127);
  pros::delay(50);

  intake_c.setDesiredVoltage(0); // stop intake
  lady_brown_motor.move_absolute(-500,
                                 200); // move lb down to score on wall stake
  timeout = 0;
  pros::delay(300); // wait for 300ms to get rings in
  while (lady_brown_motor.get_position(0) > -500 &&
         timeout < LB_TIMEOUT) { // wait for lb to finish moving
    pros::delay(10);
    timeout += 10; // increment timeout
  }
  chassis.setPose(-63, 0,
                  chassis.getPose().theta); // set position for next move

  // go to ring strip
  intake_c.setAntiJamming(true);
  intake_c.setDesiredVoltage(127); // start intake to get rings
  chassis.tank(-90, -90);          // move back to starting position
  pros::delay(300);                // wait for 300ms
  chassis.tank(0, 0);              // stop moving

  chassis.moveToPoint(-48, 0, 1200, {.forwards = false}); // move to mogo stack
  chassis.turnToHeading(180, 1200);                       // turn to mogo stack
  chassis.moveToPoint(-48, -48, 1500);                    // move to mogo stack
  chassis.moveToPoint(-48, -65, 2500, {.maxSpeed = 50});  // move to mogo stack
  chassis.waitUntilDone(); // wait for move to finish

  // get last ring
  chassis.turnToPoint(-60, -48, 1200);
  chassis.moveToPoint(-60, -48, 700); // move to last ring
  chassis.waitUntilDone();           // wait for move to finish
  pros::delay(300);                  // wait for ring to get in

  // score mogo in corner
  chassis.turnToPoint(-66, -66, 1200, {.forwards = false}); // turn to corner
  chassis.moveToPoint(-66, -66, 1200, {.forwards = false}); // move to corner
  chassis.waitUntilDone();
  mogo.set_value(0);
  chassis.tank(127, 127);
  pros::delay(300);
  chassis.tank(0, 0);

  // go to ring
  chassis.turnToPoint(-48, 0, 1200);
  chassis.moveToPoint(-48, 0, 1200, CHAIN); // move to ring
  lady_brown_motor.move_absolute(15, 200);

  // go to mid mogo

  intake_c.setDesiredVoltage(127);
  chassis.turnToHeading(45, 400);
  chassis.moveToPoint(-24, 24, 1200); // move to blue stake
  chassis.waitUntilDone();
  pros::delay(100);
  intake_c.setDesiredVoltage(0);

  chassis.moveToPoint(0, 48, 1200, {.forwards = false});
  chassis.moveToPoint(0, 48, 1200, {.forwards = false, .maxSpeed = 60});
  chassis.waitUntilDone();
  chassis.tank(-50, -50);
  pros::delay(400);
  mogo.set_value(1);
  intake_c.setDesiredVoltage(127);
  pros::delay(300);

  // throw mogo
  chassis.tank(0, -127);
  pros::delay(600);
  chassis.tank(0, 0);
  mogo.set_value(0);

  // get first mogo
  chassis.turnToPoint(-24, 48, 1200, CHAIN);
  chassis.moveToPoint(-24, 48, 1200,
                      CHAIN); // move to ring
  chassis.turnToPoint(-60, 60, 1200, CHAIN);
  chassis.moveToPoint(-60, 60, 2200, {.minSpeed = 127}); // move to ring
  chassis.waitUntilDone();
  chassis.tank(-90, -90);
  pros::delay(300);
  chassis.tank(0, 0);

  // move to 2nd mogo
  chassis.turnToPoint(-36, 36, 1200, CHAIN);
  chassis.moveToPoint(-36, 36, 1200, CHAIN);
  chassis.turnToPoint(24, 60, 1200, CHAIN);
  chassis.moveToPoint(24, 60, 1200, CHAIN); // move to 2nd mogo
  chassis.moveToPoint(66, 66, 2200, {.minSpeed = 127});

  // done

  // comment trap */
}

std::string auto_names[] = {
    "Right Red 4 Ring", "Full Lateral", "Skills Auto", "Match ladder",
    "2 Ring Left Blue", "Single Turn",  "Multi Turn"};

int num_autos = 7;
