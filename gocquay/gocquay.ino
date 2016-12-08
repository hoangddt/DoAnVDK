#include<MPU_REG.h>
/*
   HoangDDT code
*/
#include <PID_v1.h>
PID *pid;
double Kp = 300;
double Kd = 0.13;
double Ki = 30;
double Output;
double SetPoint;
byte check_Pid_Created = 0;

struct Wheel
{
  byte pin1;
  byte pin2;
  byte speedPin; // must be PWM pin

  byte pin1Value;
  byte pin2Value;
  byte speedValue;
};

/* pin1, pin 2, analog pin, value 1, value 2, speed (255) */
Wheel left = {12, 13, 11, 0, 1, 130},
      right = {4, 5, 3, 0, 1, 130}; // mark as black

/* one wheel functionality */
void registerWheel(Wheel *a);
void executeWheel(Wheel *a);
void wheelGoUp(Wheel *w);
void wheelGoDown(Wheel *w);
void stopWheel(Wheel *w);
void reverseWheel(Wheel *w);
void setSpeedByPercent(Wheel *w, byte speed);
void setSpeedByPWM(Wheel *w, byte speed);

int check_y = 0;
float Y_ondinh_Max = 0;
float Y_ondinh_Min = 0;
float Y_ondinh_TB;
byte percent_upto;
byte pecent_downto;
byte default_speed = 30;
float delta_y;

void setup()
{
  setSpeedByPercent(&left, default_speed);
  setSpeedByPercent(&right, default_speed);
  registerWheel(&left);
  registerWheel(&right);
  uint8_t c;
  int error;

  Serial.begin(19200);
  /*
    Serial.println(F("InvenSense MPU-6050"));
    Serial.println(F("June 2012"));
  */
  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();


  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  /*
    Serial.print(F("WHO_AM_I : "));
    Serial.print(c,HEX);
    Serial.print(F(", error = "));
    Serial.println(error,DEC);
  */

  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
  /*
    Serial.print(F("PWR_MGMT_2 : "));
    Serial.print(c,HEX);
    Serial.print(F(", error = "));
    Serial.println(error,DEC);
  */

  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

  //Initialize the angles
  calibrate_sensors();
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}


void loop()
{
  int error;
  accel_t_gyro_union accel_t_gyro;

  // Read the raw values.
  error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);

  // Get the time of reading for rotation computations
  unsigned long t_now = millis();



  // Convert gyro values to degrees/sec
  float FS_SEL = 131;

  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro) / FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro) / FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro) / FS_SEL;


  // Get raw acceleration values
  //float G_CONVERT = 16384;
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;

  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180 / 3.14159;
  //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;

  float accel_angle_z = 0;

  // Compute the (filtered) gyro angles
  float dt = (t_now - get_last_time()) / 1000.0;
  float gyro_angle_x = gyro_x * dt + get_last_x_angle();
  float gyro_angle_y = gyro_y * dt + get_last_y_angle();
  float gyro_angle_z = gyro_z * dt + get_last_z_angle();

  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x * dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y * dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z * dt + get_last_gyro_z_angle();

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
  float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);


  if (check_y == 0) {
    Y_ondinh_Max = Y_ondinh_Min = angle_y;
    check_y++;
    return;
  }
  if (check_y < 1000) {
    check_y++;
    Y_ondinh_Max = Y_ondinh_Max < angle_y ? angle_y : Y_ondinh_Max;
    Y_ondinh_Min = Y_ondinh_Min > angle_y ? angle_y : Y_ondinh_Min;
    return;
  }

  Y_ondinh_TB = (Y_ondinh_Max + Y_ondinh_Min) / 2;
  SetPoint = Y_ondinh_TB;
  if (!check_Pid_Created) {
    pid = new PID((double*)&angle_y, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);
    check_Pid_Created = 1;
  }
  /*
     process speed
  */
  pid->Compute();
  int currSpeed;
  currSpeed =(int) pid->GetOutput()/4;
//  if(abs(currSpeed) < 50) currSpeed = 50;

  Serial.println(currSpeed);

  byte ControllerDirection = SetPoint - angle_y > 0 ? 0 : 1;
  pid->SetControllerDirection(ControllerDirection);

  
  setSpeedByPWM(&left, abs(currSpeed));
  setSpeedByPWM(&right, abs(currSpeed));

  if (angle_y > Y_ondinh_Min && angle_y < Y_ondinh_Max && abs(currSpeed) < 15)
  {
    stopWheel(&left);
    stopWheel(&right);
    pid->SetMode(0);
  }
  else 
{
    pid->SetMode(1);
    if (pid->GetDirection())
    {
      wheelGoUp(&right);
      wheelGoUp(&left);

    }
    else
    {
      wheelGoDown(&left);
      wheelGoDown(&right);
    }
  }
  Serial.print(F("#FIL:"));             //Filtered angle
  Serial.print(angle_x, 2);
  Serial.print(F(","));
  Serial.print(angle_y, 2);
  Serial.print(F(","));
  Serial.print(angle_z, 2);
  Serial.println(F(""));

  // Delay so we don't swamp the serial port
  delay(5);
}

/*
   Hoang Code

*/
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
  setSpeedByPWM(w, speed);
}

void setSpeedByPWM(Wheel *w, byte speed)
{
  if ( (speed < 0) || (speed > 255) )
    speed = 255;
  w->speedValue = speed;
   executeWheel(w);
}
