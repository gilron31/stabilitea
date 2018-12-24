 /* Read Quadrature Encoder
   Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.

   Sketch by max wolf / www.meso.net
   v. 0.1 - very basic functions - mw 20061220

   brown - Vcc, blue - GRD, black - input

*/

#include <Encoder.h>
#include "A4988.h"

// Callback methods prototypes
void calculateDelay();
void moveMotor();


int encoder0PinA = 2;
int encoder0PinB = 3;


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(encoder0PinA, encoder0PinB);

volatile char encoder0Pos = 0;
volatile byte encoder0PinALast = LOW;
volatile byte n = LOW;

double v = 0;
double maxV = 0;
double minV = 0;
double a = 0;

int delayTime = 1000;
int maxDelayTime = 5000;
int minDelayTime = 700;

double angle = 0;
double emergancyStopAng = 10;

double location = 0;
double maxLocation = 7;

int MOTOR_CONST = 31400;
double MOTOR_STEP = 0.0314;
int SCALE = 1024*3;
int cyclesToRecalculate = 10;

int cycleCounter = 10;

int counter = 0;

byte dir;

unsigned long last_calc;

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for cruise speed
#define RPM 120
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 2000
#define MOTOR_DECEL 2000

// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 16

#define MS1 10
#define MS2 11
#define MS3 12
A4988 stepper(MOTOR_STEPS, 4, 5, 6, MS1, MS2, MS3);

void setup() {
  //pinMode (encoder0PinA, INPUT);
  //pinMode (encoder0PinB, INPUT);
  delay(2000);
  //attachInterrupt(digitalPinToInterrupt(encoder0PinA), count, CHANGE);

  /**pinMode(6,OUTPUT); // Enable
  pinMode(5,OUTPUT); // Step
  pinMode(4,OUTPUT); // Dir
  digitalWrite(6,LOW); // Set Enable low
  digitalWrite(4,HIGH); // Set Dir high
  dir = HIGH;*/
  
  stepper.begin(RPM, MICROSTEPS);
  stepper.enable();
  
  Serial.begin(115200);

  maxV = (double)MOTOR_CONST/((double)minDelayTime*1.0);
  minV = (double)MOTOR_CONST/((double)maxDelayTime*1.0);
  last_calc = micros();
}

void loop() 
{
  
   if(cycleCounter == cyclesToRecalculate)
   {
      calculateDelay();
      cycleCounter = 0;
   }
   cycleCounter += 1;
   stepper.setSpeedProfile(stepper.LINEAR_SPEED, a, -a);
   
   moveMotor();

}

/**
 * no longer relavant
 */
double speedByAngle(double angle)
{
  double rangeV = maxV - minV;
  double maxAng = 2.0;
  double tolerance = 0;
  double rangeA = maxAng - tolerance;
  double returnVal = (rangeV/rangeA)*(abs(angle)-tolerance) + minV;
  if(returnVal > maxV)
  {
    returnVal = maxV;
  }
  return returnVal;
}

/**
 * calculate the accelaration neccesary given the current measurements
 */
double accelByAngle(double angle, double da, double location)
{
  /**
  double maxAccel = 50.0;
  double maxAngle = 5.0;
  double maxAngVal = 30.0;
  double maxVal = maxAngle + 20*maxAngVal;
  double val = angle + 20*da;
  double returnVal = (maxAccel/maxVal)*val;
  */
  // pid calculation
  double returnVal = 50*(angle+location/50) + 2*da;
  if(returnVal < 0 && dir == HIGH)
  {
    digitalWrite(4,LOW);
    dir = LOW;
  }
  if(returnVal > 0 && dir == LOW)
  {
    digitalWrite(4,HIGH);
    dir = HIGH;
  }
  return abs(returnVal);
}

/**
 * move a step and update location
 */
void moveMotor()
{
  if(delayTime < maxDelayTime && delayTime > minDelayTime - 1)
  {
    if(dir == LOW)
    {
      location -= MOTOR_STEP;
    }
    else
    {
      location += MOTOR_STEP;
    }
    digitalWrite(5,HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(5,LOW);
    delayMicroseconds(delayTime);
  }
}


/**
 * calculates the delay for the motors based on measurements
 */
void calculateDelay()
{
  unsigned long now = micros();
  // read angle using encoder
  double new_angle = ((double)myEnc.read()/(SCALE*1.0))*360.0;
  // calc angle dev
  // TODO: make it using mpu gyro
  double angle_dev = (new_angle - angle)/(double)(now - last_calc)*1000000.0;
  angle = new_angle;
  
  //v = speedByAngle(measurements);

  // calc the current accelaration
  a = accelByAngle(angle,angle_dev,location);
  v += a*(double)(now - last_calc)/1000000.0;
  // limit stepper velocity
  if(v > maxV)
  {
    v = maxV;
  }
  last_calc = now;
  // transform speed to delay
  delayTime = (int)((double)MOTOR_CONST/v);

  // pause in case of crash
  if(abs(angle) > emergancyStopAng)
  {
    delayTime = 10000;
  }
/**
    Serial.print("angle: ");
    Serial.print(angle);   
    Serial.print(" velocity: ");
    Serial.print(v);
    Serial.print(" delay time: ");
    Serial.println(delayTime);
    */

}
