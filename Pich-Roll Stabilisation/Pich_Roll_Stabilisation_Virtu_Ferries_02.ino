/* Stabilisation of pitch and roll using trim tabs and t-foils. Measurements taken from a 6dof IMU.
Input 1 - PIN 2 - Stabilisation system ON/OFF - Channel 5 on RC controller
Input 2 - PIN 3 - Set running trim; position 1 = -2mm and position 2 = +2mm - Channel 6 on RC controller
Outputs:
Pin 9  - Port servo
Pin 10 - Starboard servo
Pin 11 - Forward port servo
Pin 12 - Forward starboard servo

Last edit: 03/05/16 by RS*/

// Compare to GX1
// Tune pitch rate gain by using pot; put roll gains to 0
// Tune roll gain and rate gain with pot; put pitch gains to 0

#include <Wire.h> //The I2C library
#include <Servo.h> 

#define RECEIVER_SIGNAL_IN_1 0       // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define RECEIVER_SIGNAL_IN_2 1       // INTERRUPT 1 = DIGITAL PIN 3 - use the interrupt number in attachInterrupt
#define RECEIVER_SIGNAL_IN_1_PIN 2   // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead
#define RECEIVER_SIGNAL_IN_2_PIN 3   // INTERRUPT 0 = DIGITAL PIN 3 - use the PIN number in digitalRead
#define NEUTRAL_POSITION 1500        // this is the duration in microseconds of neutral channel on an electric RC Car

volatile int nChannel1In = NEUTRAL_POSITION;     // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriod1 = 0;       // set in the interrupt
volatile boolean bNewChannelSignal1 = false;     // set in the interrupt and read in the loop; low (OFF) is <1500 and high (ON) is >1500
volatile int nChannel2In = NEUTRAL_POSITION;     // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriod2 = 0;       // set in the interrupt
volatile boolean bNewChannelSignal2 = false;     // set in the interrupt and read in the loop

Servo portServo;     //Creates Servo objects    
Servo starboardServo;    
Servo fwdPortServo; 
Servo fwdStarboardServo;  


double pitchRateGain = 80; //
//float rollGain = 1.0;
float rollRateGain = 0.2; //
float timeStep = 0.02;
unsigned long timer;
float pitchRate;
float rollRate;
float portServoSetting;
float starboardServoSetting;
float fwdPortServoSetting;  //new
float fwdStarboardServoSetting; //new

//port servo setting
int psTTabRange = 13.59;                                              //range of the trim tab in degrees 20
int psTTabCentrePosition = 18.2;                                      //measured in mm from the transom to the tip of horizontal bar
int psTTabAngleCorrection = 0.495;

//starboard servo setting
int stbTTabRange = 15.59;                                             //range of the trim tab in degrees 20
int stbTTabCentrePosition = 20.85;                                    //measured in mm from the transom to the tip of horizontal ba
int stbTTabAngleCorrection = 0.725;

//fwd port servo setting
int psTFoilRange = 10;                                                //range of the t-foil in degrees  30
int psTFoilCentrePosition = 41;                                       //measured in mm down from the keel to the rare tip of the t-foil wing
int psTFoilAngleCorrection = 0.5;

//fwd starboard servo setting
int stbTFoilRange = 10.5;                                             //range of the t-foil in degrees  30
int stbTFoilCentrePosition = 41.5;                                    //measured in mm down from the keel to the rare tip of the t-foil wing
int stbTFoilAngleCorrection = 0.55;

int currentMode = 0;

float portTTabPosition;
float starboardTTabPosition;
float portTFoilPosition;  //new
float starboardTFoilPosition;  //new

int gyroResult[3], accelResult[3];
float biasGyroX = -44;
float biasGyroY = -15;
float biasGyroZ = 2; 
float biasAccelX = 11;
float biasAccelY = 8;
float biasAccelZ = -7;
float pitchGyro = 0;
float pitchAccel = 0;
float pitchIMU = 0; //Output of Kalman filter
float pitch;
float rollGyro = 0;
float rollAccel = 0;
float roll = 0;  //Output of Kalman filter
float giroVar = 0.1;
float deltaGiroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;


void setup() {
  int totalGyroXValues = 0;
  int totalGyroYValues = 0;
  int totalGyroZValues = 0;
  int totalAccelXValues = 0;
  int totalAccelYValues = 0;
  int totalAccelZValues = 0;
  int i;
  
  attachInterrupt(RECEIVER_SIGNAL_IN_1,calcInput1,CHANGE);
  attachInterrupt(RECEIVER_SIGNAL_IN_2,calcInput2,CHANGE);
  
  Wire.begin();                                                     //Open I2C communications as master
    

  writeTo(0x53,0x31,0x09);                                          //Set accelerometer to 11bit, +/-4g
  writeTo(0x53,0x2D,0x08);                                          //Set accelerometer to measure mode  
  writeTo(0x68,0x16,0x1D);                                          //Set gyro to +/-2000deg/sec and 98Hz low pass filter 98Hz:0x1A, 5Hz:0x1E, 10Hz:0x1D
  writeTo(0x68,0x15,0x09);                                          //Set gyro to 100Hz sample rate

  portServo.attach(9);                                              // attaches port servo on pin 9 to the servo object 
  starboardServo.attach(10);                                        // attaches starboard servo on pin 10 to the servo object   
  fwdPortServo.attach(11);                                          // attaches forward port servo on pin 11 to the servo object 
  fwdStarboardServo.attach(12);                                     // attaches forward starboard servo on pin 12 to the servo object
  
  delay(100);
 
}

void loop() {

  timer = millis();
  getGyroscopeReadings(gyroResult);
  getAccelerometerReadings(accelResult);
  
  pitchRate = -(gyroResult[0] + 44.0) / 14.375;                   //pitch rate from IMU in deg/s
  rollRate = - (gyroResult[1] + 15.0) / 14.375;
  
//  pitchRateGain = analogRead(A0) / 1023.0;
//  rollRateGain = analogRead(A1) / 1023.0;
//  
//  pitchAccel = atan2((accelResult[1] - biasAccelY) / 256, (accelResult[2] - biasAccelZ) / 256) * 360.0 / (2*PI);
//  pitchGyro = pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
//  pitchIMU = pitchIMU + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
//  pitch = pitchIMU * -1;
//  
//  rollAccel = atan2((accelResult[0] - biasAccelX) / 256, (accelResult[2] - biasAccelZ) / 256) * 360.0 / (2*PI);
//  rollGyro = rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; 
//  roll = roll - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;
//  
//  Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
//  Pxv += timeStep * Pvv;
//  Pxx += timeStep * giroVar;
//  Pvv += timeStep * deltaGiroVar;
//  
//  kx = Pxx * (1 / (Pxx + accelVar));
//  kv = Pxv * (1 / (Pxx + accelVar));
//  
//  pitchIMU += (pitchAccel - pitchIMU) * kx;
//  roll += (rollAccel - roll) * kx;
//  
//  Pxx *= (1 - kx);
//  Pxv *= (1 - kx);
//  Pvv -= kv * Pxv;
  
 if(bNewChannelSignal1)
 {

   bNewChannelSignal1 = false;     // set this back to false when we have finished with
   bNewChannelSignal2 = false;     // nChannel1In, while true, calcInput will not update nChannel1In

 }
  
  //Mode 1 - Input 1: 0 Input 2: 0
  if (nChannel1In < NEUTRAL_POSITION && nChannel2In < NEUTRAL_POSITION){  //Stand-by mode; trim tabs and t-foils fixed in position
  
  portTTabPosition = psTTabCentrePosition; //new var
  starboardTTabPosition = stbTTabCentrePosition; //new var
  
  portTFoilPosition    = psTFoilCentrePosition;  //new var
  starboardTFoilPosition = stbTFoilCentrePosition;  //new var
  
  currentMode = 1;
  }
    
  //Mode 2 - Input 1: 1 Input 2: 0
  if (nChannel1In > NEUTRAL_POSITION && nChannel2In < NEUTRAL_POSITION){  //trim tabs and t-foils compensate for pitch and roll
  portTTabPosition       = psTTabCentrePosition + pitchRate * pitchRateGain - rollRate * rollRateGain;
  
  starboardTTabPosition  = stbTTabCentrePosition + pitchRate * pitchRateGain + rollRate * rollRateGain; //new var 
  
  portTFoilPosition          = psTFoilCentrePosition - pitchRate * pitchRateGain*2 - rollRate * rollRateGain; //new 
  
  starboardTFoilPosition     = stbTFoilCentrePosition - pitchRate * pitchRateGain*2 + rollRate * rollRateGain;  //new var
  
  currentMode = 2;
  }

  if (portTTabPosition > (psTTabCentrePosition + psTTabRange/2 + psTTabAngleCorrection)) {                     //Limits of servo range:
    portTTabPosition =    psTTabCentrePosition + psTTabRange/2 + psTTabAngleCorrection; 
  }
  if (portTTabPosition < (psTTabCentrePosition - psTTabRange/2 + psTTabAngleCorrection)) {
    portTTabPosition =    psTTabCentrePosition - psTTabRange/2 + psTTabAngleCorrection;
  }    

  if (starboardTTabPosition > (stbTTabCentrePosition + stbTTabRange/2 + stbTTabAngleCorrection)) {                     //Limits of servo range:
    starboardTTabPosition =    stbTTabCentrePosition + stbTTabRange/2 + stbTTabAngleCorrection;
  }  
  if (starboardTTabPosition < (stbTTabCentrePosition - stbTTabRange/2 + stbTTabAngleCorrection)) {
    starboardTTabPosition =    stbTTabCentrePosition - stbTTabRange/2 + stbTTabAngleCorrection;
  }    
 //--------------------------------------------------------------------------  
  if (portTFoilPosition > (psTFoilCentrePosition + psTFoilRange/2 + psTFoilAngleCorrection)) {                     // Limits of servo range:
    portTFoilPosition =    psTFoilCentrePosition + psTFoilRange/2 + psTFoilAngleCorrection; 
  }
  if (portTFoilPosition < (psTFoilCentrePosition - psTFoilRange/2 + psTFoilAngleCorrection)) {  
    portTFoilPosition =    psTFoilCentrePosition - psTFoilRange/2 + psTFoilAngleCorrection;
  }    

  if (starboardTFoilPosition > (stbTFoilCentrePosition + stbTFoilRange/2 + stbTFoilAngleCorrection)) {             // Limits of servo range:
    starboardTFoilPosition =    stbTFoilCentrePosition + stbTFoilRange/2 + stbTFoilAngleCorrection;
  }
  if (starboardTFoilPosition < (stbTFoilCentrePosition - stbTFoilRange/2 + stbTFoilAngleCorrection)) { 
    starboardTFoilPosition =    stbTFoilCentrePosition - stbTFoilRange/2 + stbTFoilAngleCorrection;
  }
   
   portServoSetting = - 24.29 * portTTabPosition + 1958;                    // Calibration from servo (mm) to servo output in milliseconds
   starboardServoSetting =  29.25* starboardTTabPosition + 909.8;   
//--------------------------------------------------------------------------  
   fwdPortServoSetting = 13.39 * portTFoilPosition + 950.2;  //new
   fwdStarboardServoSetting = -12.59*starboardTFoilPosition + 1998; //new

   
   portServo.writeMicroseconds(portServoSetting);
   starboardServo.writeMicroseconds(starboardServoSetting);
//--------------------------------------------------------------------------  
   fwdPortServo.writeMicroseconds(fwdPortServoSetting); //new
   fwdStarboardServo.writeMicroseconds(fwdStarboardServoSetting);  //new
   
   
   Serial.print(pitch); //This is for analysis purpose 
   Serial.print("\t");
   Serial.print(roll,4); //2
   Serial.print("\t");
   Serial.print(pitchRate);
   Serial.print("\t");
   Serial.print(rollRate);
   Serial.print("\t");
   Serial.print(portTFoilPosition,4); //2
   Serial.print("\t");
   Serial.print(starboardTFoilPosition,4); //2
   Serial.print("\t"); 
   Serial.print(currentMode);
   Serial.print("\t");
   timer = millis() - timer;
   timer = (timeStep * 1000) - timer;  
   Serial.print('\n');  
   delay(timer);
   Serial.print(timer);
}
