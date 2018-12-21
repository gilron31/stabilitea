 /* Read Quadrature Encoder
   Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.

   Sketch by max wolf / www.meso.net
   v. 0.1 - very basic functions - mw 20061220

   brown - Vcc, blue - GRD, black - input

*/
#include <TaskScheduler.h>

#include <Encoder.h>



//gil was here

// Callback methods prototypes
void calculateDelay();
void moveMotor();

//Tasks
Task t1(1, TASK_FOREVER, &moveMotor);
Task t2(10, TASK_FOREVER, &calculateDelay);

Scheduler runner;


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

void setup() {
  //pinMode (encoder0PinA, INPUT);
  //pinMode (encoder0PinB, INPUT);
  delay(2000);
  //attachInterrupt(digitalPinToInterrupt(encoder0PinA), count, CHANGE);

  pinMode(6,OUTPUT); // Enable
  pinMode(5,OUTPUT); // Step
  pinMode(4,OUTPUT); // Dir
  digitalWrite(6,LOW); // Set Enable low
  digitalWrite(4,HIGH); // Set Dir high
  dir = HIGH;

  
  Serial.begin(115200);

  maxV = (double)MOTOR_CONST/((double)minDelayTime*1.0);
  minV = (double)MOTOR_CONST/((double)maxDelayTime*1.0);


  /**runner.init();  
  runner.addTask(t1);  
  runner.addTask(t2);
  t1.enable();
  t2.enable();**/
  last_calc = micros();
}

void loop() 
{
  /**
  runner.execute();
  */
  
   if(cycleCounter == cyclesToRecalculate)
   {
      calculateDelay();
      cycleCounter = 0;
   }
   cycleCounter += 1;
   
   moveMotor();

}

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


void calculateDelay()
{
  unsigned long now = micros();
  double new_angle = ((double)myEnc.read()/(SCALE*1.0))*360.0;
  double angle_dev = (new_angle - angle)/(double)(now - last_calc)*1000000.0;
  angle = new_angle;
  
  //v = speedByAngle(measurements);

  a = accelByAngle(angle,angle_dev,location);
  v += a*(double)(now - last_calc)/1000000.0;
  if(v > maxV)
  {
    v = maxV;
  }
  last_calc = now;
  delayTime = (int)((double)MOTOR_CONST/v);

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
