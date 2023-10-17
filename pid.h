#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <timing.h>


/*
* Velocity PID implmentation
*/
class PID {
    protected:
        bool _deadbandActive {false};
        float _cp {0};
        float _ci {0};
        float _cd {0};
        float _err[3] {0, 0, 0};  // error terms for computing cv'
        float _controlValue {0};  // current control value
        
    public:
        enum DeadbandMode {OFF, ACTIVE_ON_SETPOINT, ACTIVE_ON_LIMIT};
        DeadbandMode deadbandMode;
        float k;    // system gain 
        float kp;   // proportional gain
        float ki;   // integral gain
        float kd;   // derivative gain
        float setpoint;  // processValue target
        float deadband;  // deadband for updating
        float min;       // controlValue min
        float max;       // controlValue max
        TON timer;  // timer used to set the update rate
        const bool& deadbandActive {_deadbandActive};
        const float& cp {_cp};   // contribution from the p term
        const float& ci {_ci};   // contribution from the i term
        const float& cd {_cd};   // contribution from the d term
        const float& controlValue {_controlValue};  // exposes readonly _cv
        const float& error {_err[0]};  // error
        

        /*
        * @param TON timer
        * initilizes the class with a TON.
        * Note that every time the TON finishes,
        * The PID will sample and update.
        * 
        * To adjust the poll rate, simply 
        * modify the timers.PRE value
        */
        PID(TON timer, float setpoint, float min, float max, 
            float k=1.0, float kp=1.0, float ki=1.0, float kd=0.0,
            float deadband=1.0, DeadbandMode deadbandMode=DeadbandMode::OFF) 
            : timer {timer}, setpoint {setpoint}, min {min}, max {max}, 
              k {k}, kp {kp}, ki {ki}, kd {kd},
              deadband {deadband}, deadbandMode {deadbandMode} 
        {
            // pass
        }


        /*
        * Updates the pid. Note it has no effect unless
        * the timer has passed. If the timer's pre value
        * is set to zero, dt is set to one. 
        */
        PID& call(double processValue) {
            // note it pre = 0 it will immediatly be done
            if (timer.call(!timer.DN).DN) {
                 // update the error terms
                _err[2] = _err[1];
                _err[1] = _err[0];
                _err[0] = setpoint - processValue;
                // compute the new control value
                float pollRate = timer.PRE > 0 ? static_cast<float>(timer.PRE) : 1.0;
                _cp = k*(kp * (_err[0] - _err[1]));
                _ci = k*(ki * _err[0] * pollRate);
                _cd = k*(kd * (_err[0] - (2 * _err[1]) + _err[2]) / pollRate);

                // evaluate expression for deadband active
                _deadbandActive = (deadbandMode != DeadbandMode::OFF)
                                  && (
                                    (_deadbandActive && abs(_err[0]) < deadband)
                                    || (!_deadbandActive && ((
                                            deadbandMode == DeadbandMode::ACTIVE_ON_SETPOINT 
                                            && ( (_err[0] == 0) || ( (_err[0]/abs(_err[0])) != (_err[1]/abs(_err[1])) ))
                                        ) || (
                                            deadbandMode == DeadbandMode::ACTIVE_ON_LIMIT && abs(_err[0]) <= deadband
                                        )
                                    )
                                )
                            );

                if (!_deadbandActive) {
                    _controlValue = _controlValue + (_cp + _ci + _cd);
                    if (_controlValue < min) {_controlValue = min;}
                    if (_controlValue > max) {_controlValue = max;}
                }
            }
            return *this;
        }
};

#endif