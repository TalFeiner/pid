#ifndef positionPID_h
#define positionPID_h


class PositionPID
{
    private:
        double errorOld, integral, outputVal;
        double outputMaxValue = 4000;
        double setPoint = 0, setPointTemp = 0;
        double kp = 40, ki = 100, kd = 0;
        bool pidReset = false;
        PositionPID &calcFunc(double, double);
        void pidDir();
    public:
        bool dir = true, changeDir = false;
        PositionPID (double _setPoint, double _outputMaxValue, double _kp, double _ki, double _kd)
        {
            this->kp = _kp;
            this->ki = _ki;
            this->kd = _kd;
            this->setPoint = _setPoint;
            this->outputMaxValue = _outputMaxValue;
            this->pidReset = true;
        }
        PositionPID &setSetPoint (double _setPoint) {this->setPoint = _setPoint; return *this;}
        PositionPID &setOutputMaxValue (float _outputMaxValue) {this->outputMaxValue = _outputMaxValue; return *this;}
        PositionPID &setGains (double _kp, double _ki, double _kd) {this->kp = _kp; this->ki = _ki; this->kd = _kd; return *this;}
        PositionPID &reset(bool reset = true) {this->pidReset = reset; return *this;}
        double output() {return this->outputVal;}
        double calc(double measuredValue, double dt) {this->calcFunc(measuredValue, dt); return this->outputVal;}
};
#endif
