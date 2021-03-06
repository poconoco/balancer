
// 2022 by Leonid Yurchenko <noct at ukr.net>
// https://github.com/poconoco/balancer/
//
// Balancing bot firmware
//
// Hardware modules:
//   - Raspberry Pi Pico
//   - MPU6050 6axis motion sensor
//   - L298N DC motors controller
//   - JDY-31 bluetooth module
//
// Software modules:
//   - MPU6050 by Jeff Rowberg <jeff@rowberg.net>
//   - I2Cdev lib by Jeff Rowberg <jeff@rowberg.net>
//   - RP2040_PWM by Khoi Hoang
//   - PID based on PID by Bradley J. Snyder <snyder.bradleyj@gmail.com>
//
// Features:
//   - Use MPU6050 lib to get hardware DMP readings from MPU6050
//   - Use RP2040_PWM lib to leverage PiPico hardware PWM generators

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "RP2040_PWM.h"
#include "pid.h"
#include <Wire.h>

MPU6050 mpu;

// Mechanical config
#define MPU_ORIENTATION 1, 0, 1, 0  // MPU6050  sensor orientation quaternion: turn 1 around Y axis
#define BALANCE_PITCH -2.5;    // Target angle where bot is as close as possible to balance

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
double pitchInput = 0;
double speedInput;
double pitchFeedback = 0;
double speedSetpoint = 0;
double turn;

double feedbackMaxAngle = 2;

int btMoveForward = 0;
int btMoveTurn = 0;

//Specify the links and initial tuning parameters

// PID Args: 
// double dt, double max, double min, double Kp, double Ki, double Kd
PID pitchPid(1, 100, -100, 9, 0.016, 11000, true);
PID speedPid(1, feedbackMaxAngle, -feedbackMaxAngle, 0, 0.00006, 0.0, true);

// Servo library won't work on PiPico, and we want to leverage hardware PWM generator
RP2040_PWM sr1(15, L298N_PWM_FREQUENCY, 0);
RP2040_PWM sr2(14, L298N_PWM_FREQUENCY, 0);
RP2040_PWM sl1(12, L298N_PWM_FREQUENCY, 0);
RP2040_PWM sl2(13, L298N_PWM_FREQUENCY, 0);

RP2040_PWM led(25, 1000, 0);

UART BT(0, 1, NC, NC);

void readMovementFromBT() {
    static unsigned long lastMovementCommand = 0;
    unsigned long now = millis();
  
    // Expect joystick packet as "MX<Xcoord>Y<Ycoord>", where X and Y coords are 3 digits strings from 0 to 100. 
    // For example, middle position is sent as: "MX050Y050"
    if (BT.available() >= 9) {
        while (BT.available() && BT.read() != 'M');  // Skip to first 'M' header byte
        if (BT.available() >= 8 && BT.read() == 'X') {  // Verify the second 'X' header byte
            char rawX[4];
            char rawY[4];

            BT.readBytes(rawX, 3);
            if (BT.read() == 'Y')   // Verify middle 'Y' byte
            {
                BT.readBytes(rawY, 3);

                // Provide null terminators
                rawX[3] = '\0';
                rawY[3] = '\0';
                
                btMoveForward = atoi(rawY) - 50;
                btMoveTurn = atoi(rawX) - 50;
    
                lastMovementCommand = now;
            } else{
                Serial.println("Didn't found middle Y, skipping...");
            }
        } else {
            Serial.println("Didn't found second X, skipping...");
        }

        while (BT.available()) BT.read();  // Flush the rest of available data
    }

    // If no commands for 1/2 second, stop
    if (now - lastMovementCommand >= 500) {
        btMoveForward = 0;
        btMoveTurn = 0;
    }
}

void processMovement() {    
    speedSetpoint = map(btMoveForward, -50, 50, -10, 10);
    turn = map(btMoveTurn, -50, 50, -10, 10);
}

void processBalance() {
    pitchSetpoint = BALANCE_PITCH;
    
    pitchInput = ypr[1] * 180/M_PI + pitchFeedback;
    speedInput = pitchPid.calculate(pitchSetpoint, pitchInput);
    pitchFeedback = speedPid.calculate(speedSetpoint, speedInput);

    if (abs(pitchInput - pitchSetpoint) > 45) {
        /*
        Serial.print("pitch: \t");
        Serial.print(pitchInput);
        Serial.print(" stopping");
        Serial.print("\n");
        */
        setSpeed(0);  // Stop
    } else {
        setSpeed(speedInput);
        //Serial.println(pitchInput);
        //Serial.print('\n');
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

void setSpeed(double rawSpeed)
{
    led.setDuty(rawSpeed, true);
    
    double speed;
    const double deadBand = 35;

    if (abs(rawSpeed) > 0.001) {
        speed = deadBand + mapDouble(abs(rawSpeed), 0, 100, 0, 100 - deadBand);
        if (rawSpeed < 0)
            speed = -speed;
    } else {
        speed = rawSpeed;
    }

    int allowedTurn = abs(speed)+abs(turn) >= 100 ? 0 : turn;
  
    if (speed > 0) {
        sr1.setDuty(0, true);
        sr2.setDuty(speed+allowedTurn, true);
        sl1.setDuty(0, true);
        sl2.setDuty(speed-allowedTurn, true);
    } else if (speed < 0) {
        sr1.setDuty(-speed-allowedTurn, true);
        sr2.setDuty(0, true);
        sl1.setDuty(-speed+allowedTurn, true);
        sl2.setDuty(0, true);
    } else {  // speed == 0
        sr1.setDuty(0, true);
        sr2.setDuty(0, true);
        sl1.setDuty(0, true);
        sl2.setDuty(0, true);
    }

    //servo1.setPWM(SERVO1_PIN, 50, mapSpeedToPwm(speed), true);
    //servo2.setPWM(SERVO2_PIN, 50, mapSpeedToPwm(-1 * speed), true);
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

    Serial.println("BT initialization...");  
    BT.begin(9600);
    sleep_ms(250);
    BT.println("AT+BAUD8"); // Switch baudrate to 115200
    sleep_ms(250);
    BT.end();
    BT.begin(115200);
    
    Serial.println("BT init complete...");  
}

void loop() {
    if (mpuInterrupt) {  // wait for MPU interrupt or extra packet(s) available
        getDMP();
    } else {
        // return; 
    }
    
    processBalance();
    readMovementFromBT();
    processMovement();
}
