#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <PID_v1.h>

#define DEBUG

MPU6050 mpu;

/* Motor define*/
struct Wheel
{
  byte pin1;
  byte pin2;
  byte speedPin; // must be PWN pin

  byte pin1Value;
  byte pin2Value;
  byte speedValue;
};

/* pin1, pin 2, analog pin, value 1, value 2, speed (255) */
Wheel left = {12, 13, 11, 0, 1, 0},
      right = {4, 5, 3, 0, 1, 0}; // mark as black

/* one wheel functionality */
void registerWheel(Wheel *a);
void executeWheel(Wheel *a);
void wheelGoUp(Wheel *w);
void wheelGoDown(Wheel *w);
void stopWheel(Wheel *w);
void reverseWheel(Wheel *a);
void setSpeedByPercent(Wheel *w, byte speed);
void setSpeedByPWN(Wheel *w, byte speed);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

// Balance PID controller Definitions
#define BALANCE_KP 15                   // PID Constants
#define BALANCE_KI 90
#define BALANCE_KD 0.8
#define BALANCE_PID_MIN -255              // Define PID limits to match PWM max in reverse and foward
#define BALANCE_PID_MAX 255

#define ROTATION_KP 50
#define ROTATION_KI 300
#define ROTATION_KD 4

/* ToDo: Motor define here */
// Motor Misc
#define PWM_MIN 0
#define PWM_MAX 255
float MOTORSLACK_A=32;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B=39;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
#define MOTOR_A_PWM_MAX 255               // Compensate for differences in DC motor strength
#define MOTOR_B_PWM_MAX 255   


int MotorAspeed, MotorBspeed, MotorSlack, moveState=0, d_speed, d_dir;

double yaw, input, out, setpoint, originalSetpoint, Buffer[3];
double yinput, yout, ysetpoint, yoriginalSetpoint;

double bal_kp, bal_ki, bal_kd, rot_kp, rot_ki, rot_kd;

PID pid(&input, &out, &setpoint, BALANCE_KP, BALANCE_KI, BALANCE_KD, DIRECT);
PID rot(&yinput, &yout, &ysetpoint, ROTATION_KP, ROTATION_KI, ROTATION_KD, DIRECT);

void setup()
{
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
	/* ToDo: register our motor */
  registerWheel(&left);
  registerWheel(&right);
	init_imu();
  
	pid.SetMode(AUTOMATIC);                  //For info about these,see Arduino PID library
	pid.SetOutputLimits(-210, 210);
	pid.SetSampleTime(10);
	rot.SetMode(AUTOMATIC);
	rot.SetOutputLimits(-20, 20);
	rot.SetSampleTime(10);

	setpoint = 0;
	originalSetpoint = setpoint;
	ysetpoint = 0;
	yoriginalSetpoint = ysetpoint;

	bal_kp = BALANCE_KP;
	bal_ki = BALANCE_KI;
	bal_kd = BALANCE_KD;
	rot_kp = ROTATION_KP;
	rot_ki = ROTATION_KI;
	rot_kd = ROTATION_KD;

	pid.SetTunings(bal_kp, bal_ki, bal_kd);                      //change PID values
	rot.SetTunings(rot_kp, rot_ki, rot_kd);
}

void loop() 
{
	getvalues();        //read values from imu
	new_pid();          //call pid

  #ifdef DEBUG
  printval();
  #endif
}

void init_imu()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	mpu.initialize();
	devStatus = mpu.dmpInitialize();
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(-9);
	mpu.setYGyroOffset(-3);
	mpu.setZGyroOffset(61);
	mpu.setXAccelOffset(-449);
	mpu.setYAccelOffset(2580);
	mpu.setZAccelOffset(1259);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
       // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void getvalues()
{
     // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
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
            
        }
    yinput = ypr[0]* 180/M_PI;
    input = -ypr[1] * 180/M_PI;          //change sign if negative
}

double compensate_slack(double yOutput, double Output, bool A)
{
	// Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
	// yOutput is for left,right control
	if (A)
	{
		if (Output >= 0) 
			Output = Output + MOTORSLACK_A - yOutput;
		if (Output < 0) 
			Output = Output - MOTORSLACK_A - yOutput;
	}
	else
	{
		if (Output >= 0) 
			Output = Output + MOTORSLACK_B + yOutput;
		if (Output < 0) 
			Output = Output - MOTORSLACK_B + yOutput;
	}
	Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX); 
	return Output;
}

void printval()
{
  Serial.print(yinput);Serial.print("\t"); 
  Serial.print(yoriginalSetpoint); Serial.print("\t");
  Serial.print(ysetpoint); Serial.print("\t");
  Serial.print(yout); Serial.print("\t");Serial.print("\t");  
  Serial.print(input);Serial.print("\t");
  Serial.print(originalSetpoint); Serial.print("\t");
  Serial.print(setpoint); Serial.print("\t");
  Serial.print(out); Serial.print("\t");Serial.print("\t");
  Serial.print(MotorAspeed); Serial.print("\t");
  Serial.print(MotorBspeed); Serial.println("\t");
}

void new_pid()
{
	//Compute error
	pid.Compute();
	rot.Compute();
	// Convert PID output to motor control

	MotorAspeed = compensate_slack(yout, out, 1);
	MotorBspeed = compensate_slack(yout, out, 0);
	runTwoWheel(MotorAspeed, MotorBspeed);            //change speed
}

void runTwoWheel(int MotorAspeed, int MotorBspeed)
{
	// Set speed
	setSpeedByPWN(&left, abs(MotorAspeed));
	setSpeedByPWN(&right, abs(MotorBspeed));

	// Motor A control
	if (MotorAspeed >= 0) 
	{
		wheelGoUp(&left);
	}
	else 
	{
		wheelGoDown(&left);
	}

	// Motor B control
	if (MotorBspeed >= 0) 
	{
		wheelGoUp(&right);
	}
	else 
	{
		wheelGoDown(&right);
	}
}

/*motor definition*/
void registerWheel(Wheel *a)
{
  pinMode(a->pin1, OUTPUT);
  pinMode(a->pin2, OUTPUT);
  pinMode(a->speedPin, OUTPUT);
}

void executeWheel(Wheel *a)
{
  digitalWrite(a->pin1, a->pin1Value);
  digitalWrite(a->pin2, a->pin2Value);
  analogWrite(a->speedPin, a->speedValue);
}

void wheelGoUp(Wheel *w)
{
  w->pin1Value = HIGH;
  w->pin2Value = LOW;
  executeWheel(w);
}

void wheelGoDown(Wheel *w)
{
  w->pin1Value = LOW;
  w->pin2Value = HIGH;
  executeWheel(w);
}

void stopWheel(Wheel *w)
{
  w->pin1Value = LOW;
  w->pin2Value = LOW;
  executeWheel(w);
}

void reverseWheel(Wheel *w)
{
  w->pin1Value = 1 - w->pin1Value;
  w->pin2Value = 1 - w->pin2Value;
  executeWheel(w);
}

void setSpeedByPercent(Wheel *w, byte speed)
{
  speed = map(speed, 0, 100, 0, 255);
  setSpeedByPWN(w, speed);
}

void setSpeedByPWN(Wheel *w, byte speed)
{
  if ( (speed < 0) || (speed > 255) )
    speed = 200;
  w->speedValue = speed;
}
