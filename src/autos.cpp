#include "autos.hpp"
#include "subsystems.hpp"
#include <cmath>
#include <sys/_intsup.h>

void test_lateral(int dist = 12) {
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, dist, std::max(500, dist * 300));
}

void test_lateral_range(int start_dist = 12, int dist_increment = 6,
                        int num_tests = 5) {
  // without using test_lateral as it resets the pose
  chassis.setPose(0, 0, 0);
  for (int i = 0; i < num_tests; i++) {
    int length = start_dist + i * dist_increment;
    chassis.moveToPoint(0, length * (i & 1 - 2) /*flips between 1 and -1*/,
                        std::max(500, length * 300), {.forwards = !(i & 1)});
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
  chassis.moveToPoint(-36, -24-12*std::sqrt(3), 1200,{.forwards = false});
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

std::string auto_names[] = {"Single Lateral", "Full Lateral", "Skills Auto",
                            "Match ladder", "2 Ring Match"};

int num_autos = 5;
