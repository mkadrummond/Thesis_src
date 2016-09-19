/*
 * https://gist.github.com/bradley219/5373998
 */

#ifndef _PID_H_
#define _PID_H_

class PID
{
    public:
		// Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( float dt, double max, double min, float Kp, float Kd, float Ki );

        // Returns the manipulated variable given a setpoint and current process value
        float calculate( float setpoint, float pv );
        ~PID();

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
