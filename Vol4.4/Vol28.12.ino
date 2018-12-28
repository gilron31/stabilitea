
 /* Read Quadrature Encoder
   Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.

   Sketch by max wolf / www.meso.net
   v. 0.1 - very basic functions - mw 20061220

   brown - Vcc, blue - GRD, black - input

*/
#include <Encoder.h>
//#include <I2Cdev.h>
//#include <MPU6050.h>
#include <Wire.h>
//#include <SoftwareSerial.h>

//SoftwareSerial BTSerial(8, 9); // RX | TX
int encoder0PinA = 2;
int encoder0PinB = 3;
Encoder myEnc(encoder0PinA, encoder0PinB);
int driverDir = 4;
int driverStep = 5;
int driverEnable = 6;

const double PERM_MPU_OFFSET = 0.1;
const int MOTOR_CONST = 31400;
const int MAX_DELAY_TIME = 5000;
const int MIN_DELAY_TIME = 600;
const double MAX_V = (double)MOTOR_CONST/MIN_DELAY_TIME;
const double MIN_V = (double)MOTOR_CONST/MAX_DELAY_TIME;
const double MOTOR_STEP = 0.0314;
const int SCALE = 1024*3;
const double EMERGANCY_STOP_ANGLE = 10;  
const double MPU_CORRECTION = 2.5;
const double RAIL_LENGTH = 8;
const int MEASURES_TO_CALIBRATE = 200;
const int CYCLES_TO_RECALCULATE = 10;

double v;                 //wanted velocity [cm/sec]
double a;                 //wanted acceleration [cm/sec^2]
double angle;             //angle [degrees]
double angle_dev;
double angle_offset;      //offset at the start [degrees]
double location;          //locatiob[cm]
double last_location;     //previous location
double location_dev;      //velocity of cart[cm/sec]
double intpos;            //integral of position
int delayTime;            //delay between motor steps [microsec]
byte dir;                 //direction of motor
unsigned long last_calc;  //time of doing last control calculation [microsec]
unsigned long startCalculation, endCalculation, calcTime;


void setup() {
  
  pinMode(driverEnable,OUTPUT); 
  pinMode(driverStep,OUTPUT); 
  pinMode(driverDir,OUTPUT); 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(driverEnable,LOW); 
  digitalWrite(driverDir,HIGH); 
  //BTSerial.begin(115200);   

    
  v = 0;
  a = 0;
  angle = 0;
  angle_dev = 0;
  angle_offset = 0;
  
  location = 0;
  last_location = 0;
  location_dev = 0;
  delayTime = 0;
  dir = HIGH;
  intpos =0 ;

  Serial.begin(115200);
  delay(1000);
  Serial.println("started!!");
  last_calc = micros();

  angle_offset = 29.50;
  doBlink();
  waitToAngle();
}


void doBlink()
{
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100);
  }
}

void waitToAngle()
{
  while(abs(getAngle()) >= 0.15)
  {
    delay(100);
  }
}

void loop() 
{
  for(int i = 0; i < CYCLES_TO_RECALCULATE - 1; i++)
  {
    moveMotor();
    delayMicroseconds(delayTime);
  }
  moveMotor();
  startCalculation = micros();
  calculateDelay();
  endCalculation = micros();
  calcTime = endCalculation - startCalculation;
  if(calcTime < delayTime)
  {
    delayMicroseconds(delayTime-calcTime + 10);
  }
}

void moveMotor()
{
  if(delayTime < MAX_DELAY_TIME + 1 && delayTime > MIN_DELAY_TIME - 1)
  {
    if(dir == LOW)
    {
      location -= MOTOR_STEP;
    }
    else
    {
      location += MOTOR_STEP;
    }
    digitalWrite(driverStep,HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(driverStep,LOW);
  }
  else
  {
    delayMicroseconds(MIN_DELAY_TIME);
  }
}


void calculateDelay()
{
  // calculate angle, angular velocity, location and speed
  unsigned long now = micros();
  double timeDiff = (double)(now - last_calc)/1000000.0;
  double new_angle = getAngle();
  angle_dev = (new_angle - angle)/timeDiff;
  angle = new_angle;
  location_dev = (location - last_location)/timeDiff;
  last_location = location;
  intpos= intpos + timeDiff*location;
  //calculated wanted delay time by the previous parameters
  a = calculateAcceleration(angle,angle_dev,location,location_dev,intpos);
  v += a*timeDiff;
  v = boundV(v);
  motorDirection(v);
  delayTime = (int)((double)MOTOR_CONST/(2*abs(v)));  //!!!!!!!!!!!!!!!!
  //delayTime += locationConsideration(location,v);
  delayTime = boundDelayTime(delayTime);
  if(abs(angle) > EMERGANCY_STOP_ANGLE)
  {
    delay(20000);
  }

  last_calc = now;

  //printing data (if wanted)
  printData();
}

/**
 * gets relevent parameters on the system (P,D) and returns the wanted acceleration
 */
double calculateAcceleration(double th, double dth, double x, double dx, double intx)
{
  th = angleToRad(th);
  dth = angleToRad(dth);
  double Pconst = 5000;
  double Dconst = 500;
  double thetaXRatio = 0.3;
//  double calculatedAngle = th + sign(x)*max(sign(x)*(x-RAIL_LENGTH*PI/25.0), 0)*0.0003;
//  double calculatedAngle = th;
  double calculatedAngle = th + x*0.001; //good: 0.003/2/1
  double calculatedDev = dth + dx*0.006; // good: 0.005/6
  double acceleration = Pconst*calculatedAngle + Dconst*calculatedDev;
  return acceleration;
}


/**
 * determins the direction of the motor by the sign of the velocity
 */
double motorDirection(double v)
{
  if(v < 0 && dir == HIGH)
  {
    digitalWrite(driverDir,LOW);
    dir = LOW;
  }
  if(v > 0 && dir == LOW)
  {
    digitalWrite(driverDir,HIGH);
    dir = HIGH;
  }
}


/**
 * checks that the velocity doesn't acceed the allowed values
 */
double boundV(double v)
{
  if(abs(v) > MAX_V)
  {
    v = MAX_V*sign(v);
  }
  return v;
}

double boundDelayTime(double dT)
{
  if(dT < MIN_DELAY_TIME)
  {
    return MIN_DELAY_TIME;
  }
  if(dT > MAX_DELAY_TIME)
  {
    return MAX_DELAY_TIME + 1;
  }
  return dT;
}

int locationConsideration(double location, double v)
{
  if(abs(location) > RAIL_LENGTH*(PI/14.5) && sign(location) == sign(v))
  {
     return -180;
  }
  return 0;
}

/**
 * prints relevant data
 */
void printData()
{
    //Serial.print("angle: ");
    //Serial.print(angle);
    //Serial.print(" angle V: ");
    //Serial.print(angle_dev);    
    Serial.print(" v: ");
    Serial.print(v);
    //Serial.print(" a: ");
    //Serial.print(a);
    //Serial.print(" delay time: ");
    //Serial.print(delayTime);
    //Serial.print(" location: ");
    //Serial.print(location);
    //BTSerial.println("a");
    //BTSerial.println(angle);
    //BTSerial.println(angle_dev);
    //BTSerial.println(location);
    //BTSerial.println(location_dev);
    //BTSerial.println(a);
    //BTSerial.println(millis());
}

int sign(double x)
{
  if(x > 0)
  {
    return 1;  
  }
  return -1;
}

double angleToRad(double x)
{
  return (x*PI)/180;
}

double getAngle()
{
  return ((double)myEnc.read()/(SCALE*1.0))*360.0 + angle_offset;
}
