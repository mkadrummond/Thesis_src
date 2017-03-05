/*
 * https://gist.github.com/bradley219/5373998
 */


#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>

#include <cmath>
#include "pid.h"

using namespace std;

PID::PID( float dt, double max, double min, float Kp, float Kd, float Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

float PID::calculate( float setpoint, float pv )
{
    // Calculate error
    float error = setpoint - pv;

    // Normalise
    error = error/8000;

    // Proportional term
    float Pout = this->_Kp * error;

    // Integral term
    _integral += error * _dt;
    float Iout = _Ki * _integral;

	// Derivative term
    float derivative = (error - _pre_error) / _dt;
    float Dout = _Kd * derivative;

	// Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    // De-normalise and fit to the 12-bit input of the valve
    output = 2048*(output + 1);

    return output;
}

PID::~PID() {}

#endif
