//#include <PID_v1.h>
#include <LMotorController.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <WiFi.h>
#include <PubSubClient.h>




//#ifndef PID_v1_h
//#define PID_v1_h
#define LIBRARY_VERSION	1.0.0

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    void Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();						  //  they were created mainly for the pid front-end,
	double GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	double dispKi;				//   format for display purposes
	double dispKd;				//
    
	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	double ITerm, lastInput;

	int SampleTime;
	double outMin, outMax;
	bool inAuto;
};
//#endif



/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

//#if ARDUINO >= 100
//  #include "Arduino.h"
//#else
//  #include "WProgram.h"
//#endif

//#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
	PID::SetOutputLimits(0, 255);				//default output limit corresponds to 
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis()-SampleTime;				
    inAuto = false;
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
		
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/ 
void PID::Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
	  double input = *myInput;
      double error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
 
      /*Compute PID Output*/
      double output = kp * error + ITerm- kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
   }
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID::Initialize()
{
   ITerm = *myOutput;
   //C = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}








const char* ssid = "Kiarash phone";         // WiFi SSID
const char* password = "Kiarash1";          // WiFi Password
const char* mqtt_server = "broker.hivemq.com"; // MQTT broker
const int port = 1883;
unsigned long lastCommandTime = 0; // Tracks the last command time
const unsigned long commandTimeout = 500; // 0.5 seconds timeout

double lastTime = millis()-100;
int direction = 1;
int currentSpeed=0;

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
uint8_t ITerm;
uint8_t dInput;
uint8_t lastInput;

uint8_t error;
double outMax=255;
double outMin=0;
unsigned long now = millis();


uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

int inputCommand = 0;
int speed;

//PID
double originalSetpoint = 165;
double setpoint = originalSetpoint;
double movingAngleOffset = 0;
double input, output;
double mySetpoint=165;

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

WiFiClient espClient;
PubSubClient client(espClient);


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LEDPIN,1);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LEDPIN,0);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("rcontrol"); // Update this to your actual topic
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  pinMode(LEDPIN,OUTPUT);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
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
  
  k=165;
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Parse the received payload into an integer command
  int command = atoi((char*)payload);

  // Validate and set the command if it matches the expected values
  if (command >= 0 && command <= 4) {
    inputCommand = command;
    lastCommandTime = millis(); // Update the last command time
    Serial.print("Command Received: ");
    Serial.println(inputCommand);
  } else {
    Serial.println("Invalid command received!");
  }
}

void loop() {
  if (!dmpReady) return;

  if (!client.connected()) {
    reconnect();
    digitalWrite(LEDPIN,1);
  }
  digitalWrite(LEDPIN,0);
  client.loop();

  // Check if the command timeout has been exceeded
  if (millis() - lastCommandTime > commandTimeout) {
    inputCommand = 0; // Reset to Stop
  }

  while (!mpuInterrupt && fifoCount < packetSize){
    // Action based on received command
    switch (inputCommand) {
      case 0: // Stop
        Serial.println("Stop");
        mySetpoint = 165;
        break;
      case 1: // Forward
        Serial.println("Move Forward");
        mySetpoint = 180;
        break;
      case 2: // Backward
        Serial.println("Move Backward");
        mySetpoint = 150;
        break;
      case 3: // Right
        Serial.println("Turn Right");
        mySetpoint = 165;
        break;
      case 4: // Left
        Serial.println("Turn Left");
        mySetpoint = 165;
        break;
      default:
        break;
      }
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    /*
    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    if(timeChange>=100){
      //Compute all the working error variables//
	    double input = lastInput;
      double error = k - input;
      output+= (Ki * error);
      if(output > outMax) output= outMax;
      else if(output < outMin) output= outMin;
      double dInput = (input - lastInput);
 
      //Compute PID Output//
      double output = Kp * error + output- Kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  
      //Remember some variables for next time//
      lastInput = input;
      lastTime = now;
      }
    */
    /////////////////////////////////////////////////////////////////////////////
    motorController.move(output, MIN_ABS_SPEED);
    /*
    speed=output;
    minAbsSpeed=MIN_ABS_SPEED;
    if (speed < 0){
      direction = -1;
      speed = min(speed, -1*minAbsSpeed);
      speed = max(speed, -255);
      }
    else{
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
    */
    }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024){
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
 else if (mpuIntStatus & 0x02){
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

  

  /////////////////////////////////////////////////////////////////////////////
  //pid.Compute();
  /*
  error=k-input;
  ITerm+= (Ki * error);
  if(ITerm > outMax) ITerm= outMax;
  else if(ITerm < outMin) ITerm= outMin;
  double dInput = (input - lastInput);
 
  //Compute PID Output//
  double output = kp * error + ITerm- kd * dInput;
      
	if(output > outMax) output = outMax;
  else if(output < outMin) output = outMin;
	  
  //Remember some variables for next time//
  lastInput = input;
  lastTime = now;
  */
  /////////////////////////////////////////////////////////////////////////////
  //motorController.move(output, MIN_ABS_SPEED);
  /*
  speed=output;
  minAbsSpeed=MIN_ABS_SPEED;
  if (speed < 0){
    direction = -1;
    speed = min(speed, -1*minAbsSpeed);
    speed = max(speed, -255);
    }
  else{
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
  */
  ///////////////////////////////////////////////////////////////////////////

  //delay(100); // Reduce the frequency of processing
}
}
