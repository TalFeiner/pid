#include <PositionPID.h>

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04
#define pwmLeftPin 11
#define pwmRightPin 10
#define dirLeftPin_A 7
#define dirLeftPin_B 6
#define dirRightPin_A 5
#define dirRightPin_B 4

long duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement

float pidDuration = 0.1; //  [sec]
const int outputMaxValue = 255;  //  max value for pid output
float kp = 1, ki = 4, kd = 0;  //  pid gains (or weights)
double setPoint = 0;
unsigned long  lastTime;
bool stopFlag = false;

PositionPID pidLeft(setPoint, outputMaxValue, kp, ki, kd);
PositionPID pidRghit(setPoint, outputMaxValue, kp, ki, kd);


void motorLeft(int pwm)
{
  if(pidLeft.dir == false)
  { 
    digitalWrite(dirLeftPin_A, HIGH);
    digitalWrite(dirLeftPin_B, LOW);
    analogWrite(pwmLeftPin, pwm);
  }
  else
  {
    digitalWrite(dirLeftPin_A, LOW);
    digitalWrite(dirLeftPin_B, HIGH);
    analogWrite(pwmLeftPin, pwm);
  }
}


void motorRghit(int pwm)
{
  if(pidRghit.dir == false)
  { 
    digitalWrite(dirRightPin_A, LOW);
    digitalWrite(dirRightPin_B, HIGH);
    analogWrite(pwmRightPin, pwm);
  }
  else
  {
    digitalWrite(dirRightPin_A, HIGH);
    digitalWrite(dirRightPin_B, LOW);
    analogWrite(pwmRightPin, pwm);
  }
}


void stop()
{
  motorLeft(0);
  motorRghit(0);
  pidRghit.reset();
  pidLeft.reset();
  delay(200);
}


void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  pinMode(dirLeftPin_A, OUTPUT);
  pinMode(dirLeftPin_B, OUTPUT);
  pinMode(dirRightPin_A, OUTPUT);
  pinMode(dirRightPin_B, OUTPUT);
  lastTime = millis();
}


void loop()
{
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = (duration * 0.034 / 2) / 100; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.println((String)(distance,8) + " [m]");
  
  double newSetPoint = 0.2;  //  [m]
  double dt = (double)(lastTime - millis());
  dt = dt / 1000;   //  [sec]
  if (stopFlag == true)
  {
    stopFlag = false;
    stop();
  }
  else if (dt > pidDuration)
  {
    if (setPoint != newSetPoint)
    {
      setPoint = newSetPoint;
      pidLeft.setSetPoint(setPoint);
      pidRghit.setSetPoint(setPoint);
    }
    double outputLeft = pidLeft.calc(distance, dt);
    double outputRghit = pidRghit.calc(distance, dt);
    motorLeft(outputLeft);
    motorRghit(outputRghit);
  }
  delay(1);
}
