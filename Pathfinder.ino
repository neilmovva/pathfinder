/*
Code/project by Neil Movva.

Documentation in progress.

Major thanks to Jeff Rowberg for his I2C libraries and
pioneering work with the MPU-6050. All related code is
his, or at least draws heavily from it.
*/

#define DEBUG_PRINT_YPR
#define DEBUG_PRINT_MM

//#define ARM_MOTOR
//#define MOTOR_NO_H_NFET
#define MOTOR_H_L9110

#define MPU_ENABLE
//#define MPU_ONLY_DEBUG
#define LOW_POWER //disables vibration during DROPOUT. useful for demos
#define FORCE_RECALIBRATION

#define SET(x,y) (x |= (1<<y))
#define CLR(x,y) (x &= (~(1<<y)))

#define PORTl PORTC
#define DDRl DDRC
#define L0 0
#define L1 1
#define L2 2

#define PORTm PORTD
#define DDRm DDRD
#define MP 6    //PD6
#define MN 5    //PD5

#define PORTus PORTB
#define TRIG 0    //PB0, also pin 8
#define ECHO 7    //PD7, also pin 7

#define L0a A0
#define L1a A1

#define MPa 6
#define MNa 5
#define FWD 1
#define REV -1
#define STOP 0
#define COAST -1
#define BRAKE 1
#define PULSE_LENGTH 30

#define TRIGa 8
#define ECHOa 7

#define EE_CALIBRATED_ADDR  0x01
#define EE_CALIBRATED_TEST  0xAA //arbitrary byte to write and verify for
#define EE_GYRO_X_ADDR      0x10
#define EE_GYRO_Y_ADDR      0x20
#define EE_GYRO_Z_ADDR      0x30
#define EE_ACCEL_X_ADDR     0x40
#define EE_ACCEL_Y_ADDR     0x50
#define EE_ACCEL_Z_ADDR     0x60

#define V_SOUND 0.34        //represented in mm/uS
#define MAX_RANGE 3000      //in mm
#define PING_TIMEOUT 20000  // 2 * MAX_RANGE/V_SOUND
#define MAX_HAPTIC 2000
#define DROPOUT 150
#define minPingPeriod 200

uint32_t lastFlash = 0;
uint32_t lastPing = 0;
uint32_t lastPulse = 0;
uint32_t timeElapsed = 0;

int16_t pulsePeriod = 250;
int16_t distance = MAX_RANGE;

bool faceDown = false;
bool ledState = false;


#ifdef MPU_ENABLE

#include "EEPROM.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"
#define HOST_DMP_READ_RATE 7    // 1khz / (1 + READ_RATE) = 125 Hz
#include "libs/Pathfinder_MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor 
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor 
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container gravity
int xAngle, yAngle, zAngle;

int buffersize=1000;     //
int accel_deadzone=8;     //
int gyro_deadzone=1;     //

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int16_t ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup_mpu(){
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  //data from G2-200 (how specific? very specific)
  calibrateMPU();
  
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(
      F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}

void processMPU() {
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yAngle = ypr[1] * 180 / M_PI;

    if(yAngle < -55){
      faceDown = true;
      //Serial.println("faceDown!");
    } else {
      faceDown = false;
    }

    #ifdef DEBUG_PRINT_YPR
    xAngle = ypr[0] * 180 / M_PI;
    zAngle = ypr[2] * 180 / M_PI;
    Serial.print("ypr\t\t");
    Serial.print(xAngle);
    Serial.print("\t");
    Serial.print(yAngle);
    Serial.print("\t");
    Serial.println(zAngle);
    #endif
  }
}

void avgSensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void cycleOffsets(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    avgSensors();
    Serial.println("...");

    if (abs(mean_ax)<=accel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/accel_deadzone;

    if (abs(mean_ay)<=accel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/accel_deadzone;

    if (abs(16384-mean_az)<=accel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/accel_deadzone;

    if (abs(mean_gx)<=gyro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(gyro_deadzone+1);

    if (abs(mean_gy)<=gyro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(gyro_deadzone+1);

    if (abs(mean_gz)<=gyro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(gyro_deadzone+1);

    if (ready==6) break;
  }
}

//TODO: bring in calibration routines, but in a modular way
void calibrateMPU(){
  byte isCalibrated = EEPROM.read(EE_CALIBRATED_ADDR);
  #ifdef FORCE_RECALIBRATION
  isCalibrated = false;
  #endif
  if(isCalibrated == EE_CALIBRATED_TEST){
    Serial.println("Recalling offsets from memory");
    ax_offset = EEPROM.read(EE_ACCEL_X_ADDR);
    ay_offset = EEPROM.read(EE_ACCEL_Y_ADDR);
    az_offset = EEPROM.read(EE_ACCEL_Z_ADDR);
    gx_offset = EEPROM.read(EE_GYRO_X_ADDR);
    gy_offset = EEPROM.read(EE_GYRO_Y_ADDR);
    gz_offset = EEPROM.read(EE_GYRO_Z_ADDR);
    
  } else {
    Serial.println("First run, calculating offsets");
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    avgSensors();
    cycleOffsets();

    EEPROM.write(EE_ACCEL_X_ADDR, ax_offset);
    EEPROM.write(EE_ACCEL_Y_ADDR, ay_offset);
    EEPROM.write(EE_ACCEL_Z_ADDR, az_offset);
    EEPROM.write(EE_GYRO_X_ADDR, gx_offset);
    EEPROM.write(EE_GYRO_Y_ADDR, gy_offset);
    EEPROM.write(EE_GYRO_Z_ADDR, gz_offset);
    EEPROM.write(EE_CALIBRATED_ADDR, EE_CALIBRATED_TEST); //set flag for successful calibration
  }

  Serial.print(ax_offset); 
  Serial.print("\t");
  Serial.print(ay_offset); 
  Serial.print("\t");
  Serial.print(az_offset); 
  Serial.print("\t");
  Serial.print(gx_offset); 
  Serial.print("\t");
  Serial.print(gy_offset); 
  Serial.print("\t");
  Serial.println(gz_offset);

  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
}

#endif

void drive(int cmd){
  #ifdef ARM_MOTOR
    #ifdef MOTOR_NO_H_NFET
      switch(cmd){
        case 1:   //drive motor in the (arbitrarily) forward direction
        SET(PORTm, MP);
        break;

        default:   //cut-off motor (floating)
        CLR(PORTm, MP);
        break;
      }
    #else 
      #ifdef MOTOR_H_L9110
        switch(cmd){
          case 1:   //drive motor in the (arbitrarily) forward direction
          SET(PORTm, MP);
          CLR(PORTm, MN);
          break;

          case -1:  //drive motor in the (arbitrarily) reverse direction
          CLR(PORTm, MP);
          SET(PORTm, MN);
          break;

          default:   //L9110 goes into HiZ
          CLR(PORTm, MP);
          CLR(PORTm, MN);
          break;
        }
      #endif
    #endif
  #endif
}

void drive(int cmd, int duration){
  drive(cmd);
  delay(duration);
  drive(STOP);
}

void drive(int cmd, int duration, bool coastMode){
  drive(cmd);
  if(coastMode != COAST){
    drive(STOP);
  }
}

int16_t getDistance(){
  if((millis() - lastPing) < minPingPeriod){
    return distance;
  }

  SET(PORTl, L1);

  lastPing = millis();
  int16_t mm;
  
  CLR(PORTus, TRIG);
  delayMicroseconds(5);
  SET(PORTus, TRIG);
  delayMicroseconds(10);
  CLR(PORTus, TRIG);

  mm = pulseIn(ECHOa, HIGH, PING_TIMEOUT); //MAX_RANGE roundtrip timeout

  if (mm == 0){
    return MAX_RANGE;
  }

  mm *= V_SOUND;
  mm /= 2;

  if (mm <= DROPOUT){
    return -1;
  }

  distance = mm;
  #ifdef DEBUG_PRINT_MM
  Serial.print("distance:\t");
  Serial.println(mm);
  #endif
  CLR(PORTl, L1);
  return mm;
}

void pulse(){
  drive(FWD, PULSE_LENGTH);   //drive ERM
  drive(REV, PULSE_LENGTH/4); //active braking (regen?) with default braking at end
  lastPulse = millis();
}

void translate(){
  if(distance == -1){
    while(distance == -1){
      #ifndef LOW_POWER
      drive(FWD);
      #endif
      distance = getDistance();
    }
    drive(STOP);
    return;
  }
  pulsePeriod = map(distance, DROPOUT, MAX_HAPTIC, 50, 1000);
}

void setup() {
  SET(DDRm, MP);
  SET(DDRm, MN);
  CLR(DDRm, MP);
  CLR(DDRm, MN);

  pinMode(TRIGa, OUTPUT);
  pinMode(ECHOa, INPUT);

  SET(DDRl, L0);  // the activity led (blinks when sensors are read)
  SET(DDRl, L1);  // the system status led (if off, system off)
  SET(DDRl, L2);

  digitalWrite(L0a, HIGH);

  Serial.begin(115200);

  #ifdef MPU_ENABLE
  Wire.begin();
  TWBR = 24;
  setup_mpu();
  #endif

}

void loop() {
  #ifdef MPU_ENABLE
  if(mpuInterrupt || fifoCount > packetSize){
      processMPU();
  }
  #endif
  
  #ifdef MPU_ONLY_DEBUG
  return;
  #endif

  if(faceDown){
    SET(PORTl, L2);
    //return;
  } else {
    CLR(PORTl, L2);
  }

  distance = getDistance();          //getDistance will limit itself if we're going too fast
  translate();            //expensive math op to generate pulse period value
  timeElapsed = millis() - lastPulse;  //timestamping for pulse schedule
  if(timeElapsed > pulsePeriod){  //watchdog for pulse frequency
    pulse();
  }
}