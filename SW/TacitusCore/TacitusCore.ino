#include <SoftwareSerial.h>

#include <Wire.h>

#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>


#include <I2Cdev.h>



// define I/O pins
#ifdef TEENSY
  const int ALPHA = 15;       // This is the [pinky] motor, but not really. It's the gate pin of an N-channel MOSFET, which switches a 5v motor.
#else
  const int ALPHA = 11;       // This is the [pinky] motor, but not really. It's the gate pin of an N-channel MOSFET, which switches a 5v motor.
  const int RXPIN = 8;
  const int TXPIN = 9;
  SoftwareSerial softSerial(RXPIN, TXPIN);
#endif

// Haptic feedback controls
unsigned long lastPulse;
int pulseDelay;                                         //  the delay between haptic pulses. A lower value (more frequent pulsing) feels more intense
int pulseDuration;                                      //  the duration of each pulse. A higher value (longer pulses) feels more intense
// constants for the most intense feedback (ms times)
const int minDelay = 60;
const int maxDuration = 30;
boolean dropout = false;
// constants for the softest feedback (ms times)
const int maxDelay = 750;
const int minDuration = 45;

// System Ranging Controls
const int maxRange = 300;                     // upper bound for range, sensor's validated max is ~350cm


// Sensor Controls
unsigned long lastPing = 0;                 // a timestamp-style counter that keeps the distance readings on schedule.
int pingDelay = 50;                         // time between distance measurements, defines the "schedule"
int rawDistance;

// environmental constants
const int US_ROUNDTRIP_CM = 59;   // (20000.0)/(331.3 + 0.606 * celsiusTemp) stored as uS/cm int, double DIV op takes ~34uS vs ~1uS for int
const int timeout = US_ROUNDTRIP_CM * (maxRange + 100);    // the longest time we can reasonably expect the sensor to return a value in. (includes 100 cm margin of error)

//IMU Stuff
MPU6050 mpu;
Quaternion q;


void setup()
{
  Wire.begin();
  
  Serial.begin(57600);      // higher baud rates use FEWER CPU cycles
  
  #ifdef TEENSY
    Serial3.begin(9600);
  #else
    softSerial.begin(9600);
  #endif
  
  pinMode(ALPHA, OUTPUT);
  pinMode(BETA, OUTPUT);
  
  lastPulse = 0;            // the sensor has never fired
  rawDistance = maxRange;
  
  Serial.println("Setting up MPU...");
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu.setSleepEnabled(false);;
  
}

/*
  The loop function separately maintains our inputs and outputs (ultrasonic sensor and haptic motors, respectively).
 The first and highest priority loop defines a rigid interval on which distance measurements are taken. It guarantees our data input stream.
 The second loop is best defined as an output processor. It reads the flags appropriate to the feedback motors and drives them accordingly.
 */
void loop()
{

  if(millis() - lastPing > pingDelay)                           
  {
    lastPing = millis();
    digitalWrite(ALPHA, LOW);    // Shuts off motor first to prevent any EMI skew.

    rawDistance = (getDistance() / 10) ;                    
    //Serial.println(rawDistance);

  }
  distanceToHaptic();  // we need to update our pulse delay and duration before acting on it
  if(dropout){
    analogWrite(ALPHA, 110);
    return;
  }  
  else        
  {
    if(millis() - lastPulse > pulseDelay){ 
      sendPulse();
    }
  }


}


void getMotion()
{
}  


/*
watches an internal clock to determine when it is time to send a haptic pulse
 and then sends one for pulseDuration milliseconds
 */
void sendPulse()
{
  digitalWrite(ALPHA, HIGH);
  delay(pulseDuration);
  digitalWrite(ALPHA, LOW);
  // open to experimentation with placement of timestamp
  lastPulse = millis();    
}  

/*
Uses the map function to interpolates distance input into a predefined range of haptic profiles.
 reads boolean "proximalMode", which refers to the "proximal" and "distal" feedback modes available to us.
 */
void distanceToHaptic()
{
  if(rawDistance <= 0){
    pulseDelay = 32767;
    pulseDuration = 0;
    return;
  }
  if(rawDistance <= 6){
    dropout = true;
    return;
  } 
  else {
    dropout = false;

    pulseDuration = map(rawDistance, 8, threshold, maxDuration, minDuration);
    float a = 0.05; // alternatively, divide by int 20
    float smoothDelay = (rawDistance - 8) * (rawDistance - 8);
    smoothDelay *= a;
    pulseDelay = (int) smoothDelay;
    pulseDelay += minDelay;
  }
}



int getDistance(){
  byte msb;
  byte lsb;
  
  softSerial.write(0x55);       // 0x55 = decimal value 85
  msb = softSerial.read();
  lsb = softSerial.read();
  
  //Serial.println(msb);
  //Serial.println(lsb);
  
  int mmDistance = msb*256 + lsb;
  //Serial.println(mmDistance);
  
  if(mmDistance > maxRange*10)
  {
    return -1;
  }

  return mmDistance;
}











