#include <PositionPID.h>

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04
#define pwmLeftPin 11
#define pwmRightPin 10
#define dirLeftPin_1 7
#define dirLeftPin_2 6
#define dirRightPin_1 5
#define dirRightPin_2 4

long duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement

float pidDuration = 0.1;
const int outputMaxValue = 255;
float kp = 1, ki = 6, kd = 0;
double setPoint = 0;
unsigned long  lastTime;
bool stopFlag = false;

PositionPID pidLeft(setPoint, setPoint, kp, ki, kd);
PositionPID pidRghit(setPoint, setPoint, kp, ki, kd);


void motorLeft(int pwm)
{
  if(!pidLeft.dir)
  { 
    digitalWrite(dirLeftPin_1, HIGH);
    digitalWrite(dirLeftPin_2, LOW);
    analogWrite(pwmLeftPin, pwm);
  }
  else
  {
    digitalWrite(dirLeftPin_1, LOW);
    digitalWrite(dirLeftPin_2, HIGH);
    analogWrite(pwmLeftPin, pwm);
  }
}


void motorRghit(int pwm)
{
  if(!pidRghit.dir)
  { 
    digitalWrite(dirRightPin_1, LOW);
    digitalWrite(dirRightPin_2, HIGH);
    analogWrite(pwmRightPin, pwm);
  }
  else
  {
    digitalWrite(dirRightPin_1, HIGH);
    digitalWrite(dirRightPin_2, LOW);
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
  pinMode(dirLeftPin_1, OUTPUT);
  pinMode(dirLeftPin_2, OUTPUT);
  pinMode(dirRightPin_1, OUTPUT);
  pinMode(dirRightPin_2, OUTPUT);
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
  if (stopFlag)
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
