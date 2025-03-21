#ifndef LIFT_HPP
#define LIFT_HPP

#include "lemlib/pid.hpp"
#include "pros/motor_group.hpp"

class Lift {
private:
  pros::MotorGroup &m_motors;
  float gear_ratio;
  lemlib::PID m_pid; 
  enum State {
    REST,
    READY,
    DRIVER,
  };
  State m_state;
  float getTarget() const;
  float getRotation() const;
  float m_lowest;

public:
  Lift(pros::MotorGroup &motors, float gear_ratio, lemlib::PID pid);

  void update(int up_plus_down, bool rest, bool ready);
  float getError() const;
  int getState();
};

#endif // LIFT_HPP