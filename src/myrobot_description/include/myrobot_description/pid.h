#ifndef _PID_H_
#define _PID_H_

#include <iostream>
#include <cmath>

class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID(double max, double min, double Kp, double Kd, double Ki ) :
            _max(max),
            _min(min),
            _Kp(Kp),
            _Kd(Kd),
            _Ki(Ki),
            _pre_error(0),
            _integral(0)
        {

        }

        // Returns the manipulated variable given a setpoint and current process value
        // dt -  loop interval time
        double calculate(double setpoint, double pv, double dt) {
            // Calculate error
            double error = setpoint - pv;

            // Proportional term
            double Pout = _Kp * error;

            // Integral term
            _integral += error * dt;
            double Iout = _Ki * _integral;

            // Derivative term
            double derivative = (error - _pre_error) / dt;
            double Dout = _Kd * derivative;

            // Calculate total output
            double output = Pout + Iout + Dout;

            // Restrict to max/min
            if( output > _max )
                output = _max;
            else if( output < _min )
                output = _min;

            // Save error to previous error
            _pre_error = error;

            return output;
        }

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};

#endif