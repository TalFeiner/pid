#include "PositionPID.h"
#include <math.h>


void PositionPID::pidDir()
{
    bool dirTemp;
    if (this->outputVal > 0)
    {
      dirTemp = true;
    }
    else if(this->outputVal < 0)
    {
      dirTemp = false;
      this->outputVal = -this->outputVal;
    }
    else
    {
      dirTemp = this->dir;
    }

    if (dirTemp != this->dir)
    {
        this->dir = dirTemp;
        this->changeDir = true;
        this->pidReset = true;
    }
    else
    {
        this->changeDir = false;
    }
}


PositionPID &PositionPID::calcFunc(double measuredValue, double dt)
{
    double error;
    double derivative;
    bool errorCalc;
    bool integralCalc;
    bool derivativeCalc;
    if (!this->pidReset)
    {
        error = 0;
        derivative = 0;
        errorCalc = true;
        if(this->ki != 0) integralCalc = true;
        else integralCalc = false;
        if(this->kd != 0) derivativeCalc = true;
        else derivativeCalc = false;
    }
    else
    {
        this->integral = 0;
        this->errorOld = 0;
        errorCalc = false;
        derivativeCalc = false;
        integralCalc = false;
        this->pidReset = false;
    }
    if(this->setPointTemp != 0)
    {
        this->setPoint = this->setPointTemp;
        this->setPointTemp = 0;
        integralCalc = false;
    }
    if(errorCalc) error = this->setPoint - measuredValue;
    if(integralCalc) this->integral += error *  dt;
    if(derivativeCalc) derivative = (error - this->errorOld) / dt;
    this->outputVal = this->kp * error + this->ki * this->integral + this->kd * derivative;
    if(fabs(this->outputVal) > this->outputMaxValue) {
        if(this->outputVal > 0){
            this->outputVal = this->outputMaxValue;
            this->setPointTemp = this->outputMaxValue;
        }
        else {
            this->outputVal = -this->outputMaxValue;
            this->setPointTemp = -this->outputMaxValue;
        }
        if(integralCalc) {
            this->integral += (1 / this->ki) * (this->setPointTemp - (this->kp * error));
        }
    this->pidDir();
    return *this;
    }
}
