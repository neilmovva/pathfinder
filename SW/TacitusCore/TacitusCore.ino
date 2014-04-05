#include <Wire.h>

#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>


#include <I2Cdev.h>



// define I/O pins
const int ALPHA = 15;       // This is the [pinky] motor, but not really. It's the gate pin of an N-channel MOSFET, which switches a 5v motor.
const int BETA = 16;        // This is the wrist-mounted motor; the auxilary coarse feedback device.



// Haptic feedback controls
unsigned long lastPulse;
int pulseDelay;                                         //  the delay between haptic pulses. A lower value (more frequent pulsing) feels more intense
int pulseDuration;                                      //  the duration of each pulse. A higher value (longer pulses) feels more intense
// constants for the most intense feedback (ms times)
const int minDelay = 60;
boolean dropout = false;
const int maxDuration = 18;
// constants for the softest feedback (ms times)
const int maxDelay = 400;
const int minDuration = 25;

//Long Range Specific
boolean LRMode;                                   // global flag to dictate active range mode
const int minDelayLR = 100;
const int maxDelayLR = 500;

// System Ranging Controls
const int threshold = 100;                    // the switchover point between proximal and distal ranges
const int maxRange = 300;                     // upper bound for distal range, sensor's validated max is ~350cm


// Sensor Controls
unsigned long lastPing = 0;                 // a timestamp-style counter that keeps the distance readings on schedule.
int pingDelay = 50;                         // time between distance measurements, defines the "schedule"
int rawDistance;

// environmental constants
const int US_ROUNDTRIP_CM = 59;   // (20000.0)/(331.3 + 0.606 * celsiusTemp) stored as uS/cm int, double DIV op takes ~34uS vs ~1uS for int
const int timeout = US_ROUNDTRIP_CM * (maxRange + 100);    // the longest time we can reasonably expect the sensor to return a value in. (includes 100 cm margin of error)

MPU6050 mpu;


void setup()
{
  Wire.begin();
  TWBR = 24;

  
  Serial.begin(57600);      // higher baud rates use FEWER CPU cycles
  
  Serial3.begin(9600);
  
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
    analogWrite(ALPHA, 130);
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
  if(LRMode){
    digitalWrite(BETA, HIGH);
    delay(pulseDuration);
    digitalWrite(BETA, LOW);  
  } 
  else {
    digitalWrite(ALPHA, HIGH);
    delay(pulseDuration);
    digitalWrite(ALPHA, LOW);
  }  

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
  if(rawDistance <= 7){
    dropout = true;
    return;
  } 
  else {
    dropout = false;

    if(!LRMode){
      pulseDuration = map(rawDistance, 8, threshold, maxDuration, minDuration);

      float a = 0.05; // alternatively, divide by int 20
      float smoothDelay = (rawDistance - 8) * (rawDistance - 8);
      smoothDelay *= a;
      pulseDelay = (int) smoothDelay;
      pulseDelay += minDelay;
    }
    else if(LRMode){
      LRMode = true;
      pulseDuration = 30;
      pulseDelay = map(rawDistance, threshold + 1, maxRange, minDelayLR, maxDelayLR);
    }



  }
}



int getDistance(){
  byte msb;
  byte lsb;
  
  Serial3.write(0x55);       // 0x55 = decimal value 85
  msb = Serial3.read();
  lsb = Serial3.read();
  
  //Serial.println(msb);
  //Serial.println(lsb);
  
  int mmDistance = msb*256 + lsb;
  Serial.println(mmDistance);

  if(mmDistance > maxRange*10)
  {
    return -1;
  }
  
  if(mmDistance <= 1000){
     LRMode = false;
  } else {
     LRMode = true;
  }

  return mmDistance;
}











