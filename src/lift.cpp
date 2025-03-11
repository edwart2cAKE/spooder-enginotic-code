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

float Lift::getRotation() const { return m_motors.get_position();}

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
Lift::Lift(pros::MotorGroup &motors, float gear_ratio, lemlib::PID pid,
           lemlib::ExitCondition small_exit_condition =
               lemlib::ExitCondition(0.5, 100),
           lemlib::ExitCondition large_exit_condition =
               lemlib::ExitCondition(2, 500))
    : m_motors(motors), gear_ratio(gear_ratio), m_pid(pid), m_small_exit_condition(small_exit_condition),
      m_large_exit_condition(large_exit_condition) {
  m_state = REST;
}

// Public member functions

/*
up_plus_down is representing the driver's input to move the lift up or down
where 1 is up, -1 is down, and 0 is no movement ready is a boolean representing
that the driver wants to move the lift to the ready position
*/
void Lift::update(int up_plus_down, bool ready) {
  // switch state to driver if the driver wants to move the lift and the lift is in the ready position
  if (m_state == READY && up_plus_down >= 0) {
    m_state = DRIVER;
    m_pid.reset();
  }

  // switch state to rest if the driver wants to move the lift down and the lift
  // is at the ready position
  if (m_state == READY && up_plus_down < 0) {
    m_state = REST;
  }

  // switch state to ready if the driver moves the lift up from the rest
  // position
  if (m_state == REST && up_plus_down > 0) {
    m_state = READY;
  }

  // move lift according to the driver's input
  if (m_state == DRIVER) {
    m_motors.move_voltage(127 * up_plus_down);
  }

  // if the lift is a the ready or rest position, move the lift to that position
  // using the PID controller
  if (m_state == READY || m_state == REST) {
    const float error = getError();
    const float output = m_pid.update(error);
    m_motors.move_voltage(output);
  }
}
