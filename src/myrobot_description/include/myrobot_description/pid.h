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
            _e_p(0),
            _e_pp(0),
            _u_p(0)
        {

        }

        void set_params(double Kp, double Kd, double Ki) {
            _Kp = Kp;
            _Kd = Kd;
            _Ki = Ki;
        }

        // Returns the manipulated variable given a setpoint and current process value
        // dt -  loop interval time
        double calculate(double e, double dt) {
            double KI = _Kp*_Ki*dt;
            double KD = (_Kp*_Kd)/dt;

            double u = _u_p + _Kp*(e-_e_p) + KI*e + KD*(e-2*_e_p+_e_pp);

            _u_p = u;
            _e_pp = _e_p;
            _e_p = e;

            // Restrict to max/min
            if (u > _max)
                u = _max;
            else if (u < _min)
                u = _min;

            return u;
        }

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _e_p;
        double _e_pp;
        double _u_p;
};

#endif