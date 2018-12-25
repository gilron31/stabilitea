
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

#define in1 7
#define in2 8
MPU6050 accelgyro(0x68);
int16_t ax, ay, az,gx, gy, gz;

int encoder0PinA = 2;
int encoder0PinB = 3;
Encoder myEnc(encoder0PinA, encoder0PinB);

const double PERM_MPU_OFFSET = 0.1;
const int MOTOR_CONST = 31400;
const int MAX_DELAY_TIME = 3000;
const int MIN_DELAY_TIME = 600;
const double MAX_V = (double)MOTOR_CONST/MIN_DELAY_TIME;
const double MIN_V = (double)MOTOR_CONST/MAX_DELAY_TIME;
const double MOTOR_STEP = 0.0314;
const int SCALE = 1024*3;
const double EMERGANCY_STOP_ANGLE = 10;
const double MPU_CORRECTION = 2;
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

unsigned long start;      //time of starting the program [microsec]
unsigned long last_calc;  //time of doing last control calculation [microsec]

// runtime calibration variables

bool nowCalibrating = false;
bool stopCalibrating = false;
unsigned long lastCalib;


/*
double g = 980;             //moment of inertia [cm/sec^2]
double M = 0.3;             //mass of cart [Kg]
double mStick = 0.2;        //mass of stick [Kg]
double mEnd = 0.2;          //mass of top of stick [Kg]
double l = 100.0;           //length of stick [cm]
double b = 0.1;             //friction of cart [Kg/sec]
double I = mStick*l*l/12 + mStick*l*l/8 + mEnd*l/2;
double m = mStick + mEnd;
double q = ((M+m)*(I+m*l*l)-m*m*l*l);
double A = (b/q)*(I + m*l*l);
double B = -(M+m)*m*g*l/q;
double C = -b*m*g*l/q;
double D = m*l/q;
*/

void setup() {
  
  Wire.begin();
  TWBR = 24;
  accelgyro.initialize();
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //while (Serial.available() && Serial.read());
  pinMode(6,OUTPUT); // Enable
  pinMode(5,OUTPUT); // Step
  pinMode(4,OUTPUT); // Dir
  digitalWrite(6,LOW); // Set Enable low
  digitalWrite(4,HIGH); // Set Dir high

  //setting the MPU's offset according to previous calculations
  accelgyro.setXAccelOffset(-792);
  accelgyro.setYAccelOffset(717);
  accelgyro.setZAccelOffset(888);
  accelgyro.setXGyroOffset(-68);
  accelgyro.setYGyroOffset(-35);
  accelgyro.setZGyroOffset(22);
    
  v = 0;
  a = 0;
  angle = 0;
  angle_offset = 0;
  location = 0;
  last_location = 0;
  delayTime = 0;
  dir = HIGH;

  delay(5000);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("started!!");
  last_calc = micros();
  start = micros();

  startCalibrate();
}


void loop() 
{
  for(int i = 0; i < CYCLES_TO_RECALCULATE; i++)
  {
    moveMotor();
  }
  calculateDelay();
}

void startCalibrate()
{
  for(int i = 0; i < MEASURES_TO_CALIBRATE; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double temporary = atan2((double)ax,(double)ay)*(180/PI) + 180;
    angle_offset += temporary / (double)MEASURES_TO_CALIBRATE;
    delay(2);
  }
  angle_offset = angle_offset - MPU_CORRECTION;
  Serial.println(angle_offset);
  
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
    digitalWrite(5,HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(5,LOW);
    delayMicroseconds(delayTime);
  }
}



void calculateDelay()
{
  // calculate angle, angular velocity, location and speed
  unsigned long now = micros();
  double timeDiff = (double)(now - last_calc)/1000000.0;
  double new_angle = ((double)myEnc.read()/(SCALE*1.0))*360.0 + angle_offset;
  double angle_dev = (new_angle - angle)/timeDiff;
  angle = new_angle;
  double location_dev = (location - last_location)/timeDiff;
  last_location = location;

  // run time calibration

  if(!stopCalibrating && !nowCalibrating && location < RAIL_LENGTH*2.0/5.0)
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


  // end of runtime calibration


  //calculated wanted delay time by the previous parameters
  a = calculateAcceleration(angle,angle_dev,location,location_dev);
  v += a*timeDiff;
  v = boundV(v);
  motorDirection(v);
  delayTime = (int)((double)MOTOR_CONST/abs(v));
  delayTime = boundDelay(delayTime);
  if(abs(angle) > EMERGANCY_STOP_ANGLE)
  {
    double timeFromStart = (double)(now - start)/1000000.0;
    Serial.println(timeFromStart);
    delay(10000);
  }

  last_calc = now;

  //printing data (if wanted)
  printData();
}

/**
 * gets relevent parameters on the system (P,D) and returns the wanted acceleration
 */
double calculateAcceleration(double th, double dth, double x, double dx)
{
  th = angleToRad(th);
  dth = angleToRad(dth);
  double Pconst = 10000;
  double Dconst = 3000;
  double thetaXRatio = 0.3;
  double calculatedAngle = th;
  if(x > RAIL_LENGTH*(2/5) && x < RAIL_LENGTH*(3/5))
  {
    calculatedAngle += 0.00;
  }
  if(x < -RAIL_LENGTH*(2/5) && x > -RAIL_LENGTH*(3/5))
  {
    calculatedAngle -= 0.00;
  }
  double calculatedDev = dth - dx*0;
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
    digitalWrite(4,LOW);
    dir = LOW;
  }
  if(v > 0 && dir == LOW)
  {
    digitalWrite(4,HIGH);
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

int boundDelay(int dT)
{
  if(dT > MAX_DELAY_TIME)
  {
    dT = MAX_DELAY_TIME;
  }
  return dT;
}


/**
 * prints relevant data
 */
void printData()
{
  /*
    Serial.print("angle: ");
    Serial.print(angle);   
    Serial.print(" velocity: ");
    Serial.print(v);
    Serial.print(" acceleration: ");
    Serial.print(a);
    Serial.print(" delay time: ");
    Serial.println(delayTime);
    */
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
