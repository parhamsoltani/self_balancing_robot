//#include <analogWrite.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#include <WiFi.h> // Include Wi-Fi library

const char* ssid = "Parham's iPhone";         // Replace with your Wi-Fi SSID
const char* password = "levid1"; // Replace with your Wi-Fi password

//void PID::Compute()
//{
//   if(!inAuto) return;
//   unsigned long now = millis();
//   unsigned long timeChange = (now - lastTime);
//   if(timeChange>=SampleTime)
//   {
//      /*Compute all the working error variables*/
//	  double input = *myInput;
//      double error = *mySetpoint - input;
//      ITerm+= (ki * error);
//      if(ITerm > outMax) ITerm= outMax;
//      else if(ITerm < outMin) ITerm= outMin;
//      double dInput = (input - lastInput);
// 
//      /*Compute PID Output*/
//      double output = kp * error + ITerm- kd * dInput;
//      
//	  if(output > outMax) output = outMax;
//      else if(output < outMin) output = outMin;
//	  *myOutput = output;
//	  
//      /*Remember some variables for next time*/
//      lastInput = input;
//      lastTime = now;
//   }
//}
int direction = 1;
int currentSpeed=0;

AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30
int minAbsSpeed = 30; // Declare as an integer variable
#define LEDPIN 2
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t flag=0;
uint8_t k;
uint8_t kp;
uint8_t kd;
uint8_t ITerm;
uint8_t dInput;
uint8_t lastInput;
uint8_t lastTime;
uint8_t outMin;
uint8_t outMax;
uint8_t error;
unsigned long now = millis();

uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0
int inputCommand = STOP;
int speed;

//PID
double originalSetpoint = 165;
double setpoint = originalSetpoint;
double movingAngleOffset = 0;
double input, output;


//adjust these values to fit your own design
//29
double Kp = 29;   
double Kd = 5;
double Ki = 1300;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;

//MOTOR CONTROLLER
int ENA = 32;
int IN1 = 25;
int IN2 = 33;
int IN3 = 27;
int IN4 = 26;
int ENB = 14;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}


// HTML Interface
const char* htmlHomePage PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<style>
body { text-align: center; font-family: Arial, sans-serif; }
table { margin: auto; }
td { padding: 20px; background: #ccc; border: 1px solid #999; cursor: pointer; }
td:hover { background: #aaa; }
</style>
</head>
<body>
<h1>Mini Segway Control</h1>
<table>
  <tr>
    <td></td>
    <td onclick='sendButtonInput("MoveCar,1")'>Forward</td>
    <td></td>
  </tr>
  <tr>
    <td onclick='sendButtonInput("MoveCar,3")'>Left</td>
    <td onclick='sendButtonInput("MoveCar,0")'>Stop</td>
    <td onclick='sendButtonInput("MoveCar,4")'>Right</td>
  </tr>
  <tr>
    <td></td>
    <td onclick='sendButtonInput("MoveCar,2")'>Backward</td>
    <td></td>
  </tr>
</table>
<script>
function sendButtonInput(command) {
    var ws = new WebSocket('ws://' + location.hostname + '/CarInput');
    ws.onopen = function() { ws.send(command); };
    ws.onclose = function() { ws = null; };
}
</script>
</body>
</html>
)HTML";

// WebSocket Event Handler
void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                              AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->opcode == WS_TEXT) {
            String msg = String((char *)data);
            if (msg.startsWith("MoveCar,")) {
                inputCommand = msg.substring(8).toInt();
            }
        }
    }
}

// Handle Commands
void handleMovement() {
  
    switch (inputCommand) {
      Serial.println(inputCommand);
      digitalWrite(LEDPIN,1);
        case FORWARD:
            k=130;
            break;
        case BACKWARD:
            k=190;
            break;
        case LEFT:
            k=165;
            //motorController.move(80, -80); // Left motor slow, right motor fast
            break;
        case RIGHT:
            k=165;
            //motorController.move(-80, 80); // Left motor fast, right motor slow
            break;
        case STOP:
        default:
            k=165;
            //motorController.move(0, 0); // Stop
            break;
    }
}



void setup()
{
  Serial.begin(115200);
  pinMode(LEDPIN,OUTPUT);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Connected to Wi-Fi");
  Serial.println(WiFi.localIP()); // Print the IP address of the ESP32
 // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
 Wire.setClock(400000L);
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif

 mpu.initialize();

 devStatus = mpu.dmpInitialize();

 // supply your own gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(-0.21);
 mpu.setYGyroOffset(0.56);
 mpu.setZGyroOffset(0.17);
 mpu.setZAccelOffset(0.98); // 1688 factory default for my test chip

 // make sure it worked (returns 0 if so)
 if (devStatus == 0)
 {
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);

 // enable Arduino interrupt detection
 attachInterrupt(15, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();

 // set our DMP Ready flag so the main loop() function knows it's okay to use it
 dmpReady = true;

 // get expected DMP packet size for later comparison
 packetSize = mpu.dmpGetFIFOPacketSize();
 
 //setup PID
 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-255, 255); 
 }
 else
 {
 // ERROR!
 // 1 = initial memory load failed
 // 2 = DMP configuration updates failed
 // (if it's going to break, usually the code will be 1)
 //Serial.print(F("DMP Initialization failed (code "));
 //Serial.print(devStatus);
 //Serial.println(F(")"));
 }
 // Setup Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send_P(200, "text/html", htmlHomePage);
  });
  wsCarInput.onEvent(onCarInputWebSocketEvent);
  server.addHandler(&wsCarInput);
  server.begin();
  k=165;
  
}


void loop()
{
  //Serial.println(k);
 // if programming failed, don't try to do anything
 if (!dmpReady) return;

 // wait for MPU interrupt or extra packet(s) available
 while (!mpuInterrupt && fifoCount < packetSize)
 {
 //no mpu data - performing PID calculations and output to motors 
 pid.Compute();
 motorController.move(output, MIN_ABS_SPEED);
 }

 // reset interrupt flag and get INT_STATUS byte
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 // get current FIFO count
 fifoCount = mpu.getFIFOCount();

 // check for overflow (this should never happen unless our code is too inefficient)
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 // reset so we can continue cleanly
 mpu.resetFIFO();
 //Serial.println(F("FIFO overflow!"));

 // otherwise, check for DMP data ready interrupt (this should happen frequently)
 }
 else if (mpuIntStatus & 0x02)
 {
 // wait for correct available data length, should be a VERY short wait
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 // read a packet from FIFO
 mpu.getFIFOBytes(fifoBuffer, packetSize);
 
 // track FIFO count here in case there is > 1 packet available
 // (this lets us immediately read more without waiting for an interrupt)
 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI + 180;

 //for(k=165;k<180;k++){
  /////////////////////////////////////////////////////////////////////////////
  //pid.Compute();
  
  error=k-input;
  ITerm+= (Ki * error);
  if(ITerm > outMax) ITerm= outMax;
  else if(ITerm < outMin) ITerm= outMin;
  double dInput = (input - lastInput);
 
  /*Compute PID Output*/
  double output = kp * error + ITerm- kd * dInput;
      
	if(output > outMax) output = outMax;
  else if(output < outMin) output = outMin;
	  
  /*Remember some variables for next time*/
  lastInput = input;
  lastTime = now;
  /////////////////////////////////////////////////////////////////////////////
  //motorController.move(output, MIN_ABS_SPEED);
  
    speed=output;
    minAbsSpeed=MIN_ABS_SPEED;
    if (speed < 0)
    {
        direction = -1;
        
        speed = min(speed, -1*minAbsSpeed);
        speed = max(speed, -255);
    }
    else
    {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }
    
    if (speed == currentSpeed) return;
    
    int realSpeed = max(minAbsSpeed, abs(speed));
    

    if (speed > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      }
    else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      }
    
    analogWrite(ENA, realSpeed * motorSpeedFactorLeft);
    analogWrite(ENB, realSpeed * motorSpeedFactorRight);
    
    currentSpeed = direction * realSpeed;
    ///////////////////////////////////////////////////////////////////////////
    handleMovement();
    

 //}
 
 }
 
}
