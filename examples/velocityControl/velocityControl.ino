#include <PositionPID.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define encPin_A 2 
#define encPin_B 3

#define pwmPin 11
#define dirPin_A 7
#define dirPin_B 6

long oldPosition  = -0;
double angularVel = 0;
const float D = 0.1651;  //  wheel diameter [m]
const int pulsesPerRev = 60;  //  number of encoder pulses per revolve
float pidDuration = 0.1;  //  [sec]
byte outputMaxValue = 4000;  //  max value for pid output
float kp = 1, ki = 6, kd = 0;  //  pid gains (or weights)
double setPoint = 0;
unsigned long  lastTime;
bool stopFlag = false;

Encoder enc(encPin_A, encPin_B);
PositionPID pid(setPoint, outputMaxValue, kp, ki, kd);


double calcWheelVel(long newPosition, long oldPosition, double dt)  //  calculate Wheel's Angular Velocity
{
  double dtheta = (double)(2 * PI) * (double)((double)((double)newPosition - (double)oldPosition) / (double)pulsesPerRev);  //  [rad]
  return dtheta/dt;  //  [rad/sec]
}


void drive(byte pwm = 1)
{
  if (pwm != 0)
  {
    byte pwm = pid.output();
  }
  if(pid.dir == false)
  { 
    digitalWrite(dirPin_A, HIGH);
    digitalWrite(dirPin_B, LOW);
    analogWrite(pwmPin, pwm);
  }
  else
  {
    digitalWrite(dirPin_A, LOW);
    digitalWrite(dirPin_B, HIGH);
    analogWrite(pwmPin, pwm);
  }
}


void stop()
{
  drive(0);
  pid.reset();
  delay(200);
}


void setup()
{
  Serial.begin(9600); // Serial Communication is starting with 9600 of baudrate speed
  pinMode(dirPin_A, OUTPUT);
  pinMode(dirPin_B, OUTPUT);
  lastTime = millis();
}


void loop()
{
  float newkp = 1, newki = 4, newkd = 0;
  byte newoutputMaxValue = 255;
  byte newSetPoint = 100;
  double dt = (double)(lastTime - millis());
  dt = dt / 1000;   //  [sec]
  if (stopFlag)
  {
    stopFlag = false;
    stop();
  }
  if (setPoint != newSetPoint || newoutputMaxValue != outputMaxValue)
  {
    setPoint = newSetPoint;
    outputMaxValue = newoutputMaxValue;
    pid.setSetPoint(setPoint).setOutputMaxValue(outputMaxValue);
  }
  if (newkp != kp || newki !=ki || newkd != kd)
  {
    pid.setGains(newkp, newki, newkd);
    kp = newkp; ki = newki; kd =newkd;
  }
  else if (dt > pidDuration)
  {
    long newPosition = enc.read();
    angularVel = calcWheelVel(newPosition, oldPosition, dt);  //  [rad/sec]
    oldPosition = newPosition;
    Serial.println((String)(angularVel,8) + " [m]");
    double output = pid.calc(angularVel, dt);
    drive();
  }
  delay(1);
}
