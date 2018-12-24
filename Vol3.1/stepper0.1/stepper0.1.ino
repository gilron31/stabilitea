 /* Read Quadrature Encoder
   Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.

   Sketch by max wolf / www.meso.net
   v. 0.1 - very basic functions - mw 20061220

   brown - Vcc, blue - GRD, black - input

*/

//gil was here


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
  delay(2000);

  pinMode(6,OUTPUT); // Enable
  pinMode(5,OUTPUT); // Step
  pinMode(4,OUTPUT); // Dir
  digitalWrite(6,LOW); // Set Enable low
  digitalWrite(4,HIGH); // Set Dir high
  dir = HIGH;

  
  Serial.begin(115200);
}

void loop() 
{
   moveMotor();

}
void moveMotor()
{
  delayTime = 500;
    digitalWrite(5,HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(5,LOW);
    delayMicroseconds(delayTime);
  
}
