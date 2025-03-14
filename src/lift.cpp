#include "lift.hpp"
#include "lemlib/exitcondition.hpp"
#include <cstdlib>

// Private helper functions
float Lift::getTarget() const {
  switch (m_state) {
  case REST:
    return 10;
  case READY:
    return 31.5;
  case DRIVER:
    break;
  }
  return 10;
}

float Lift::getRotation() const { return m_motors.get_position(); }

float Lift::getError() const {
  float error = getTarget() - getRotation();
  return error;
}

int Lift::getState() {
  switch (m_state) {
  case REST:
    return 1;
  case READY:
    return 2;
  case DRIVER:
    return 3;
  }
  return -1;
}

// Constructor
Lift::Lift(
    pros::MotorGroup &motors, float gear_ratio, lemlib::PID pid)
    : m_motors(motors), gear_ratio(gear_ratio), m_pid(pid) {
  m_state = REST;
}

// Public member functions

/*
up_plus_down is representing the driver's input to move the lift up or down
where 1 is up, -1 is down, and 0 is no movement ready is a boolean representing
that the driver wants to move the lift to the ready position
*/
void Lift::update(int up_down, bool rest, bool ready) {
  // state changes

  // ready button -> READY
  if (ready) {
    m_state = READY;
  }

  // rest button -> REST
  if (rest) {
    m_state = REST;
  }

  // up_down + READY -> DRIVER
  if (up_down != 0 && m_state == READY) {
    m_state = DRIVER;
  }

  // pid control
  if (m_state == READY || m_state == REST) {
    const float output = m_pid.update(this->getError());
    m_motors.move(output);
  }

  // driver control
  if (m_state == DRIVER) {
    static int lady_brown_speed = 0;
    if (up_down != 0) {
      lady_brown_speed += (up_down * 10);
      m_motors.move(lady_brown_speed);
    } else {
      m_motors.brake();
      lady_brown_speed = 0;
    }
  }
}
