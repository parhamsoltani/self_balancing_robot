#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class PID {
public:
    PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection);
    void Compute();
    void SetTunings(double Kp, double Ki, double Kd);
    void SetSampleTime(int NewSampleTime);
    void SetOutputLimits(double Min, double Max);
    void SetMode(int Mode);
    void SetControllerDirection(int Direction);

private:
    double *myInput, *myOutput, *mySetpoint;
    double kp, ki, kd;
    double dispKp, dispKi, dispKd;
    double ITerm, lastInput;
    unsigned long lastTime;
    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto;
    int controllerDirection;
};

#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define MIN_ABS_SPEED 30

// MPU6050 and motor control variables
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID and motor control variables
double originalSetpoint = 165;
double setpoint = originalSetpoint;
double input, output;
double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;

// Motor pins
const int ENA = 32;
const int IN1 = 25;
const int IN2 = 33;
const int IN3 = 27;
const int IN4 = 26;
const int ENB = 14;

// Create PID instance
PID pid(&input, &output, &setpoint, 29, 1300, 5, DIRECT);

// MPU interrupt detection routine
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

// Implementation of PID methods (same as in your first code)
[Previous PID implementation methods remain the same]

class LMotorController {
public:
    LMotorController(int ena, int in1, int in2, int enb, int in3, int in4, double speedFactorLeft, double speedFactorRight) {
        _ena = ena;
        _in1 = in1;
        _in2 = in2;
        _enb = enb;
        _in3 = in3;
        _in4 = in4;
        _speedFactorLeft = speedFactorLeft;
        _speedFactorRight = speedFactorRight;
        
        pinMode(_ena, OUTPUT);
        pinMode(_in1, OUTPUT);
        pinMode(_in2, OUTPUT);
        pinMode(_enb, OUTPUT);
        pinMode(_in3, OUTPUT);
        pinMode(_in4, OUTPUT);
    }

    void move(int leftSpeed, int rightSpeed, int minAbsSpeed) {
        if (leftSpeed < 0) {
            leftSpeed = min(leftSpeed, -minAbsSpeed);
            leftSpeed = max(leftSpeed, -255);
            digitalWrite(_in1, LOW);
            digitalWrite(_in2, HIGH);
            analogWrite(_ena, abs(leftSpeed) * _speedFactorLeft);
        } else {
            leftSpeed = max(leftSpeed, minAbsSpeed);
            leftSpeed = min(leftSpeed, 255);
            digitalWrite(_in1, HIGH);
            digitalWrite(_in2, LOW);
            analogWrite(_ena, leftSpeed * _speedFactorLeft);
        }

        if (rightSpeed < 0) {
            rightSpeed = min(rightSpeed, -minAbsSpeed);
            rightSpeed = max(rightSpeed, -255);
            digitalWrite(_in3, LOW);
            digitalWrite(_in4, HIGH);
            analogWrite(_enb, abs(rightSpeed) * _speedFactorRight);
        } else {
            rightSpeed = max(rightSpeed, minAbsSpeed);
            rightSpeed = min(rightSpeed, 255);
            digitalWrite(_in3, HIGH);
            digitalWrite(_in4, LOW);
            analogWrite(_enb, rightSpeed * _speedFactorRight);
        }
    }

    void move(int speed, int minAbsSpeed) {
        move(speed, speed, minAbsSpeed);
    }

private:
    int _ena, _in1, _in2, _enb, _in3, _in4;
    double _speedFactorLeft, _speedFactorRight;
};

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000L);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // Set gyro offsets
    mpu.setXGyroOffset(-0.21);
    mpu.setYGyroOffset(0.56);
    mpu.setZGyroOffset(0.17);
    mpu.setZAccelOffset(0.98);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(15, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        // Initialize PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    }
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180/M_PI + 180;
    }
}
