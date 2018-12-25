/* Read Quadrature Encoder
   Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.

   Sketch by max wolf / www.meso.net
   v. 0.1 - very basic functions - mw 20061220

   brown - Vcc, blue - GRD, black - input

*/
#include <Encoder.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>

//int MPUin1 = 7;
//int MPUin2 = 8;
//MPU6050 accelgyro(0x68);
//int16_t ax, ay, az,gx, gy, gz;
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
double angle_offset;      //offset at the start [degrees]
double location;          //locatiob[cm]
double last_location;     //velocity of cart[cm/sec]
int delayTime;            //delay between motor steps [microsec]
byte dir;                 //direction of motor
bool nowCalibrating;
bool stopCalibrating;
unsigned long lastCalib;
unsigned long start;      //time of starting the program [microsec]
unsigned long last_calc;  //time of doing last control calculation [microsec]


void setup() {
  
  //Wire.begin();
  //TWBR = 24;
  //accelgyro.initialize();
  //pinMode(MPUin1, OUTPUT);
  //pinMode(MPUin2, OUTPUT);
  pinMode(driverEnable,OUTPUT); 
  pinMode(driverStep,OUTPUT); 
  pinMode(driverDir,OUTPUT); 
  pinMode(LED_BUILTIN, OUTPUT);
  //while (Serial.available() && Serial.read());
  //digitalWrite(MPUin1, LOW);
  //digitalWrite(MPUin1, HIGH);
  digitalWrite(driverEnable,LOW); 
  digitalWrite(driverDir,HIGH); 

  //setGyroOffsets();
    
  v = 0;
  a = 0;
  angle = 0;
  angle_offset = 0;
  location = 0;
  last_location = 0;
  delayTime = 0;
  dir = HIGH;
  nowCalibrating = false;
  stopCalibrating = false;

  Serial.begin(115200);
  delay(1000);
  //Serial.println("started!!");
  last_calc = micros();
  start = micros();

  //startCalibrate();
  angle_offset = 28.05;
  doBlink();
  waitToAngle();
}

/*

void setGyroOffsets()
{
  accelgyro.setXAccelOffset(-792);
  accelgyro.setYAccelOffset(717);
  accelgyro.setZAccelOffset(888);
  accelgyro.setXGyroOffset(-68);
  accelgyro.setYGyroOffset(-35);
  accelgyro.setZGyroOffset(22);
}
*/

/*
void startCalibrate()
{
  for(int i = 0; i < MEASURES_TO_CALIBRATE; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double temporary = atan2((double)ax,(double)ay)*(180/PI) + 180;
    angle_offset += temporary / (double)MEASURES_TO_CALIBRATE;
    delay(2);
  }
  if(angle_offset > 300)
  {
    angle_offset = angle_offset - 360;
  }
  angle_offset = angle_offset - MPU_CORRECTION;
  Serial.println("angle offset by MPU :");
  Serial.println(angle_offset);
}
*/

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
  while(abs(getAngle()) >= 0.2)
  {
    delay(100);
  }
}

void loop() 
{
  for(int i = 0; i < CYCLES_TO_RECALCULATE; i++)
  {
    moveMotor();
  }
  calculateDelay();
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
    delayMicroseconds(delayTime);
  }
  else
  {
    delayMicroseconds(MIN_DELAY_TIME*2);
  }
}


void calculateDelay()
{
  // calculate angle, angular velocity, location and speed
  unsigned long now = micros();
  double timeDiff = (double)(now - last_calc)/1000000.0;
  double new_angle = getAngle();
  double angle_dev = (new_angle - angle)/timeDiff;
  angle = new_angle;
  double location_dev = (location - last_location)/timeDiff;
  last_location = location;

/*
  // run time calibration
  if(!stopCalibrating && !nowCalibrating && abs(location) < RAIL_LENGTH*2.0/5.0)
  {
    nowCalibrating = true;
    lastCalib = now;
  }
  else if(!stopCalibrating && nowCalibrating && abs(location) > RAIL_LENGTH*7.0/10.0)
  {
    nowCalibrating = false;
    angle_offset += 0.1*sign(location);
    Serial.print("new angle offset is: ");
    Serial.println(angle_offset);
  }
  else if(!stopCalibrating && nowCalibrating && double(now - lastCalib)/1000000.0 >= 5.0)
  {
    stopCalibrating = true;
    Serial.println("#################################");
    Serial.print("final angle offset is: ");
    Serial.println(angle_offset);
    Serial.println("#################################");
  }
*/
  
  //calculated wanted delay time by the previous parameters
  a = calculateAcceleration(angle,angle_dev,location,location_dev);
  v += a*timeDiff;
  v = boundV(v);
  motorDirection(v);
  delayTime = (int)((double)MOTOR_CONST/abs(v));
  //delayTime += locationConsideration(location,v);
  delayTime = boundDelayTime(delayTime);
  if(abs(angle) > EMERGANCY_STOP_ANGLE)
  {
    double timeFromStart = (double)(now - start)/1000000.0;
    //Serial.print("time of session: ");
    //Serial.println(timeFromStart);
    delay(20000);
  }

  last_calc = now;

  //printing data (if wanted)
  //printData();
  //sendData();
}

/**
 * gets relevent parameters on the system (P,D) and returns the wanted acceleration
 */
double calculateAcceleration(double th, double dth, double x, double dx)
{
  th = angleToRad(th);
  dth = angleToRad(dth);
  double Pconst = 10000;
  double Dconst = 1000;
  double thetaXRatio = 0.3;
  double calculatedAngle = th + sign(x)*max(sign(x)*(x-RAIL_LENGTH*PI/25.0), 0)*0.0003;
  double calculatedDev = dth - dx*0.00;
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
 * sends data using bluetooth
 */
void sendData()
{
  Serial.println('a');
  Serial.println(angle);
  Serial.println(v);
  Serial.println(a);
  Serial.println(location);
  Serial.println(millis());
}

/**
 * prints relevant data
 */
void printData()
{
    Serial.print("angle: ");
    Serial.print(angle);   
    Serial.print(" velocity: ");
    Serial.print(v);
    Serial.print(" acceleration: ");
    Serial.print(a);
    Serial.print(" delay time: ");
    Serial.print(delayTime);
    Serial.print(" location: ");
    Serial.println(location);
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
