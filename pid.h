/*
 * https://gist.github.com/bradley219/5373998
 */

#ifndef _PID_H_
#define _PID_H_

class PID {
public:
	PID( float dt, double max, double min, float Kp, float Kd, float Ki );
    ~PID();

    float calculate( float setpoint, float pv );

private:
    float _dt;
    double _max;
    double _min;
    float _Kp;
    float _Kd;
    float _Ki;
    float _pre_error;
    float _integral;
};

#endif
