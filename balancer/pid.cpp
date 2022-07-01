/**
 * Original by Bradley J. Snyder <snyder.bradleyj@gmail.com>
 * 
 * Modified by Leonid Yurchenko (https://github.com/poconoco/)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include <Arduino.h>
#include "pid.h"

using namespace std;

#define BUFFER_SIZE 3

class PIDImpl
{
    public:
        PIDImpl(double dt, double max, double min, double Kp, double Ki, double Kd, bool windupProtection);
        ~PIDImpl();
        double calculate(double setpoint, double pv);

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Ki;
        double _Kd;
        bool _windupProtection;
        double _lastError;
        double _integral;
        double _lastOutput;
        double _valueBuffer[BUFFER_SIZE];
        int _valueBufferPointer;
};


PID::PID(double dt, double max, double min, double Kp, double Ki, double Kd, bool windupProtection)
{
    pimpl = new PIDImpl(dt,max,min,Kp,Ki,Kd,windupProtection);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Ki, double Kd, bool windupProtection) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Ki(Ki),
    _Kd(Kd),
    _windupProtection(windupProtection),
    _lastError(0),
    _integral(0),
    _lastOutput(0),
    _valueBufferPointer(0)
{
  for (int i = 0; i < BUFFER_SIZE; i++)
    _valueBuffer[i] = 0;
}

double PIDImpl::calculate(double setpoint, double pv)
{   
    // Save to round buffer
    if (_valueBufferPointer >= BUFFER_SIZE)
        _valueBufferPointer = 0;
    _valueBuffer[_valueBufferPointer++] = pv;

    // Average of round buffer
    double smoothedValue = 0;
    for (int i = 0; i < BUFFER_SIZE; i++)
        smoothedValue += _valueBuffer[i];
    smoothedValue /= BUFFER_SIZE;
  
    // Calculate error
    double error = setpoint - smoothedValue;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    if (! _windupProtection || (_min < _lastOutput && error < 0) || (_max > _lastOutput && error > 0)) {

        // Zero integral if we crossed setpoint (?)
//        if (error < 0 && _integral > 0 || error > 0 && _integral < 0)
//          _integral = 0;
      
        _integral += error * _dt;
    }
    double Iout = _Ki * _integral;

    /*
    Serial.print(_Ki, 2);
    Serial.print("\t");
    Serial.print(_integral, 2);
    Serial.print("\t");
    Serial.print(Iout, 2);
    Serial.print("\n");
    */

    // Derivative term
    double derivative = (error - _lastError) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    /*
    Serial.print(Pout, 2);
    Serial.print("\t");
    Serial.print(Iout, 2);
    Serial.print("\t");
    Serial.print(Dout, 2);
    Serial.print("\n");
    */
    
    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save last and output
    _lastError = error;
    _lastOutput = output;
    
    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
