#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

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
Wheel left = {12, 13, 11, 0, 1, 100},
      right = {4, 5, 3, 0, 1, 100}; // mark as black

/* one wheel functionality */
void registerWheel(Wheel *a);
void executeWheel(Wheel *a);
void wheelGoUp(Wheel *w);
void wheelGoDown(Wheel *w);
void stopWheel(Wheel *w);
void reverseWheel(Wheel *a);
void setSpeedByPercent(Wheel *w, byte speed);
void setSpeedByPWN(Wheel *w, byte speed);


void setup()
{
  registerWheel(&left);
  registerWheel(&right);

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

void loop()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // display tab-separated accel/gyro x/y/z values
//  Serial.print("a/g:\t");
//  Serial.print(ax); Serial.print("\t");
//  Serial.print(ay); Serial.print("\t");
//  Serial.print(az); Serial.print("\t");
//  Serial.print(gx); Serial.print("\t");
//  Serial.print(gy); Serial.print("\t");
//  Serial.println(gz);
  //

  if (gy > 113)
  {
    wheelGoUp(&left);
    wheelGoUp(&right);
   // delay(100);
  }
  else if (gy < 49)
  {
    wheelGoDown(&left);
    wheelGoDown(&right);
    //delay(100);
  }
  else
  {
    stopWheel(&left);
    stopWheel(&right);
    //(30);
  }

  //    wheelGoUp(&left);
  //    wheelGoUp(&right);
  //    delay(100);
  //    stopWheel(&left);
  //    stopWheel(&right);
  //    delay(30);
  //    wheelGoDown(&left);
  //    wheelGoDown(&right);
  //    delay(100);
}

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

