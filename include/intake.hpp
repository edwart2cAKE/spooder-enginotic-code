#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "lemlib/timer.hpp"

class Intake {
public:
    Intake(pros::Motor& motor, pros::Optical& colorSensor);
    void controlIntake();
    void setColorRange(int min, int max); // set color range for ring detection
    void controlIntakeTask(); // start the intake control task
    void setDesiredVoltage(int voltage) { desired_voltage = voltage; } // set desired voltage for the motor
    void setAntiJamming(bool enable) { anti_jamming = enable; } // enable/disable anti-jamming

private:
    pros::Motor& motor;
    pros::Optical& colorSensor;
    bool jammed;
    lemlib::Timer jam_timer;
    lemlib::Timer anti_jam_timer;
    lemlib::Timer color_sort_timer;
    lemlib::Timer color_sort_stop_timer;
    bool was_ring_detected;
    bool task_created; // flag to check if the task is created
    bool anti_jamming = true; // flag to enable/disable anti-jamming

    int min_color; // min color value for ring detection
    int max_color; // max color value for ring detection

    int desired_voltage; // desired voltage for the motor
};

#endif // INTAKE_HPP