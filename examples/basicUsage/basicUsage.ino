#include <PositionPID.h>

#define inputPin 0
#define pwmPin 3

float pidDuration = 0.1;
double measuredValue = 0;
const int outputMaxValue = 255;
float kp = 1, ki = 6, kd = 0;
double setPoint = 0;
unsigned long  lastTime;

PositionPID pid(setPoint, measuredValue, kp, ki, kd);


void setup()
{
  lastTime = millis();
}


void loop()
{
  double newSetPoint = 100;
  double dt = (double)(lastTime - millis());
  dt = dt / 1000;   //  [sec]
  if (dt > pidDuration)
  {
    measuredValue = analogRead(inputPin);
    if (setPoint != newSetPoint)
    {
      setPoint = newSetPoint;
      pid.setSetPoint(setPoint);
    }
    double outputValue = pid.calc(measuredValue, dt);
    analogWrite(pwmPin, outputValue);
  }
  delay(1);
}
