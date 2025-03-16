#include "intake.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"

Intake::Intake(pros::Motor &motor, pros::Optical &colorSensor)
    : motor(motor), colorSensor(colorSensor), jammed(false), jam_timer(500),
      anti_jam_timer(200), color_sort_timer(30), color_sort_stop_timer(200),
      was_ring_detected(false), min_color(-1), max_color(-1),
      task_created(false), desired_voltage(127), anti_jamming(true) {
  colorSensor.set_led_pwm(100); // set the LED to full brightness
}

void Intake::controlIntake() {
  bool is_blue =
      max_color > colorSensor.get_hue() && colorSensor.get_hue() > min_color;

  if (motor.get_actual_velocity() > -10 && motor.get_actual_velocity() < 10 &&
      motor.get_current_draw() > 0.7 * motor.get_current_limit() && anti_jamming) {
    jam_timer.resume();
    if (jam_timer.isDone()) {
      jammed = true;
      anti_jam_timer.reset();
    }
  } else if (anti_jam_timer.isDone()) {
    // reset jam timers if the motor is moving again
    jam_timer.pause();
    jam_timer.reset();
    anti_jam_timer.reset();
    anti_jam_timer.pause();
    jammed = false;
  }

  if (!jammed) {
    if (was_ring_detected && !is_blue) {
      color_sort_timer.resume();
    }
    if (color_sort_timer.isDone() && !color_sort_stop_timer.isDone()) {
      motor.move(0);
      color_sort_stop_timer.resume();
    }
    if (color_sort_stop_timer.isDone()) {
      color_sort_timer.pause();
      color_sort_timer.reset();
      color_sort_stop_timer.reset();
      color_sort_stop_timer.pause();
    }
  } else {
    motor.move(-127);
    anti_jam_timer.resume();
  }
  was_ring_detected = is_blue;
  //pros::lcd::print(7, "Color Sort Timer %d", color_sort_timer.getTimeLeft());
}

void Intake::setColorRange(int min, int max) {
  min_color = min;
  max_color = max;
}

void Intake::controlIntakeTask() {
  if (!task_created) {
    pros::Task([this]() {
      while (true) {
        motor.move(desired_voltage); // set motor to desired voltage
        colorSensor.set_led_pwm(100); // set the LED to full brightness
        controlIntake(); // will stop the motor if jammed or color sort is
                         // active automatically
        pros::delay(10); // small delay to prevent task from hogging CPU
      }
    });
    task_created = true;
  }
}


