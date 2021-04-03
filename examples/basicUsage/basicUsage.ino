#include <PositionPID.h>

#define inputPin 3
#define pwmPin 11

float pidDuration = 0.1;  //  [sec]
double measuredValue = 0;
const int outputMaxValue = 255;  //  max value for pid output
float kp = 1, ki = 4, kd = 0;  //  pid gains (or weights)
double setPoint = 0;
unsigned long  lastTime;

PositionPID pid(setPoint, outputMaxValue, kp, ki, kd);


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
