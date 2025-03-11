#ifndef LIFT_HPP
#define LIFT_HPP

#include "lemlib/exitcondition.hpp"
#include "lemlib/pid.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include <string>

class Lift {
private:
  pros::MotorGroup &m_motors;
  float gear_ratio;
  lemlib::PID m_pid;
  lemlib::ExitCondition m_small_exit_condition;
  lemlib::ExitCondition m_large_exit_condition;
  enum State {
    REST,
    READY,
    DRIVER,
  };
  State m_state;
  float getTarget() const;
  float getRotation() const;

public:
  Lift(pros::MotorGroup &motors, float gear_ratio, lemlib::PID pid,
       lemlib::ExitCondition small_lift_exit_condition,
       lemlib::ExitCondition large_exit_condition);

  void update(int up_plus_down, bool ready);
  float getError() const;
  int getState();
};

#endif // LIFT_HPP