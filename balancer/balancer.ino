
// 2022 by Leonid Yurchenko
// https://www.youtube.com/@nocomake
//
// Balancing bot firmware

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "RP2040_PWM.h"
#include "pid.h"
#include "bt-remote/receivers/cpp/BtRcReceiver.h"

#include <Wire.h>

MPU6050 mpu;

// Mechanical config
#define MPU_ORIENTATION 1, 0, 1, 0  // MPU6050  sensor orientation quaternion: turn 1 around Y axis
#define BALANCE_PITCH -3.0;    // Target angle where bot is as close as possible to balance

// Electrical config
#define INTERRUPT_PIN 2
#define L298N_PWM_FREQUENCY 20000

// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
Quaternion orientation(MPU_ORIENTATION);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//PID ins and outs
double pitchSetpoint = BALANCE_PITCH;
double pitchFeedback = 0;

double feedbackMaxAngle = 2;

//Specify the links and initial tuning parameters

// PID Args: 
// double dt, double max, double min, double Kp, double Ki, double Kd
PID pitchPid(1, 100, -100, 9, 0.02, 4000, true);
PID speedPid(1, feedbackMaxAngle, -feedbackMaxAngle, 0.0, 0.00010, 0.0, true);

// Servo library won't work on PiPico, and we want to leverage hardware PWM generator
RP2040_PWM sr1(15, L298N_PWM_FREQUENCY, 0);
RP2040_PWM sr2(14, L298N_PWM_FREQUENCY, 0);
RP2040_PWM sl1(12, L298N_PWM_FREQUENCY, 0);
RP2040_PWM sl2(13, L298N_PWM_FREQUENCY, 0);

RP2040_PWM led(25, 1000, 0);

class MovementRC {
  public:
    MovementRC(BtRcReceiver<UART> *btRC,
               float maxAbsSpeedControl,
               float maxAbsTurnControl,
               float idleTurnTune) 
      : _btRC(btRC)
      , _maxAbsSpeedControl(maxAbsSpeedControl)
      , _maxAbsTurnControl(maxAbsTurnControl)
      , _idleTurnTune(idleTurnTune) {
    }

    float getSpeed() {
      return _speed;
    }

    float getTurn() {
      return _turn;
    }

    // Should be called periodically, to not let the BT serial buffer overflow
    void tick() {
      _btRC->tick();
      _speed = map(_btRC->getY1(), -128, 127, -_maxAbsSpeedControl, _maxAbsSpeedControl);
      _turn = map(_btRC->getX1(), -128, 127, -_maxAbsTurnControl, _maxAbsTurnControl) + _idleTurnTune;
    }

  private:
    BtRcReceiver<UART> *_btRC;
    double _speed;
    double _turn;

    double _maxAbsSpeedControl;
    double _maxAbsTurnControl;
    double _idleTurnTune;
};

UART btUart(0, 1, NC, NC);
BtRcReceiver<UART> btRC(&btUart, "Noco BALANCER", "1234");
MovementRC movementRC(&btRC,
                      40,   // Max abs speed control
                      20,   // Max abs turn control
                      -2);  // Idle turn tune

void processBalance() {
    pitchSetpoint = BALANCE_PITCH;

    float pitchInput = ypr[1] * 180/M_PI + pitchFeedback;
    float speedInput = pitchPid.calculate(pitchSetpoint, pitchInput);
    pitchFeedback = speedPid.calculate(movementRC.getSpeed(), speedInput);

    if (abs(pitchInput - pitchSetpoint) > 45) {
        setSpeed(0);  // Stop
    } else {
        setSpeed(speedInput);
    }
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void i2cSetup() {
    Wire.begin();
    Wire.setClock(400000);
}

void MPU6050Connect() {
  static int MPUInitCntr = 0;

  mpu.initialize();
  const uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  Serial.println(F("Attaching interrupt..."));
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, FALLING);
  Serial.println(F("Attached"));  

  mpu.getIntStatus();  // Do we need this?
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
}

void getDMP() { // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();
  /*
  fifoCount is a 16-bit unsigned value. Indicates the number of bytes stored in the FIFO buffer.
  This number is in turn the number of bytes that can be read from the FIFO buffer and it is
  directly proportional to the number of samples available given the set of sensor data bound
  to be stored in the FIFO
  */

  // PacketSize = 42; refference in MPU6050_6Axis_MotionApps20.h Line 527
  // FIFO Buffer Size = 1024;
  uint16_t MaxPackets = 20;// 20*42=840 leaving us with  2 Packets (out of a total of 24 packets) left before we overflow.
  // If we overflow the entire FIFO buffer will be corrupt and we must discard it!

  // At this point in the code FIFO Packets should be at 1 99% of the time if not we need to look to see where we are skipping samples.
  if ((fifoCount % packetSize) || (fifoCount > (packetSize * MaxPackets)) || (fifoCount < packetSize)) { // we have failed Reset and wait till next time!
    Serial.println(F("Reset FIFO"));
    if (fifoCount % packetSize) Serial.println(F("\tPacket corruption")); // fifoCount / packetSize returns a remainder... Not good! This should never happen if all is well.
    Serial.print(F("\tfifoCount ")); Serial.println(fifoCount);
    Serial.print(F("\tpacketSize ")); Serial.println(packetSize);

    const uint8_t mpuIntStatus = mpu.getIntStatus(); // reads MPU6050_RA_INT_STATUS       0x3A
    Serial.print(F("\tMPU Int Status ")); Serial.println(mpuIntStatus , BIN);
    // MPU6050_RA_INT_STATUS       0x3A
    //
    // Bit7, Bit6, Bit5, Bit4          , Bit3       , Bit2, Bit1, Bit0
    // ----, ----, ----, FIFO_OFLOW_INT, I2C_MST_INT, ----, ----, DATA_RDY_INT

    /*
    Bit4 FIFO_OFLOW_INT: This bit automatically sets to 1 when a FIFO buffer overflow interrupt has been generated.
    Bit3 I2C_MST_INT: This bit automatically sets to 1 when an I2C Master interrupt has been generated. For a list of I2C Master interrupts, please refer to Register 54.
    Bit1 DATA_RDY_INT This bit automatically sets to 1 when a Data Ready interrupt is generated.
    */
    if (mpuIntStatus & B10000) { //FIFO_OFLOW_INT
      Serial.println(F("\tFIFO buffer overflow interrupt "));
    }
    if (mpuIntStatus & B1000) { //I2C_MST_INT
      Serial.println(F("\tSlave I2c Device Status Int "));
    }
    if (mpuIntStatus & B1) { //DATA_RDY_INT
      Serial.println(F("\tData Ready interrupt "));
    }
    Serial.println();
    //I2C_MST_STATUS
    //PASS_THROUGH, I2C_SLV4_DONE,I2C_LOST_ARB,I2C_SLV4_NACK,I2C_SLV3_NACK,I2C_SLV2_NACK,I2C_SLV1_NACK,I2C_SLV0_NACK,
    mpu.resetFIFO();// clear the buffer and start over
    mpu.getIntStatus(); // make sure status is cleared we will read it again.
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      if (fifoCount < packetSize) break; // Something is left over and we don't want it!!!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    if (fifoCount > 0) mpu.resetFIFO(); // clean up any leftovers Should never happen! but lets start fresh if we need to. this should never happen.
  }
}


void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  gravity.rotate(&orientation);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  //Yaw = (ypr[0] * 180 / M_PI);
  //Pitch = (ypr[1] * 180 / M_PI);
  //Roll = (ypr[2] * 180 / M_PI);
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double mapSpeedToPwm(double speed) {
  return mapDouble(speed, -100, 100, 5, 10);
}

double constrainSpeedControl(double speedControl)
{
    if (speedControl < 0)
        return 0;

    if (speedControl > 100)
        return 100;

    return speedControl;
}

void setSpeed(double rawSpeed)
{
    led.setDuty(abs(rawSpeed), true);
    
    double speed;
    const double deadBand = 35;

    if (abs(rawSpeed) > 0.001) {
        speed = deadBand + mapDouble(abs(rawSpeed), 0, 100, 0, 100 - deadBand);
        if (rawSpeed < 0)
            speed = -speed;
    } else {
        speed = rawSpeed;
    }

    float turn = movementRC.getTurn();

    if (speed == 0) 
    {
        sr1.setDuty(0, true);
        sr2.setDuty(0, true);
        sl1.setDuty(0, true);
        sl2.setDuty(0, true);    
    } 
    else
    {        
        double rSpeed = speed+turn;
        if (rSpeed > 0)
        {
            sr1.setDuty(0, true);
            sr2.setDuty(constrainSpeedControl(rSpeed), true);
        }
        else
        {
            sr1.setDuty(constrainSpeedControl(-rSpeed), true);
            sr2.setDuty(0, true);
        }

        double lSpeed = speed-turn;
        if (lSpeed > 0)
        {
            sl1.setDuty(0, true);
            sl2.setDuty(constrainSpeedControl(lSpeed), true);
        }
        else
        {
            sl1.setDuty(constrainSpeedControl(-lSpeed), true);
            sl2.setDuty(0, true);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println(F("Start"));

    sr1.setPWM();
    sr2.setPWM();
    sl1.setPWM();
    sl2.setPWM();

    i2cSetup();
    Serial.println(F("i2cSetup complete"));
    MPU6050Connect();
    Serial.println(F("MPU6050Connect complete"));

    btRC.init();
}

void loop() {
    if (mpuInterrupt) {  // wait for MPU interrupt or extra packet(s) available
        getDMP();
    } else {
        // return; 
    }
    
    processBalance();
    movementRC.tick();
}
