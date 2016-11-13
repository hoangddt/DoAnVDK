#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t min_ax, min_ay, min_az;
int16_t max_ax, max_ay, max_az;

int16_t min_gx, min_gy, min_gz;
int16_t max_gx, max_gy, max_gz;

int16_t gx, gy, gz;
int speed_test;
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

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
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  min_ax = max_ax = ax; min_ay = max_ay = ay; min_az = max_az = az;
  min_gx = max_gx = gx; min_gy = max_gy = gy; min_gz = max_gz =gz;
}

void loop()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  min_ax = min_ax > ax ? ax : min_ax;
  max_ax = max_ax < ax ? ax : max_ax;
  min_ay = min_ay > ay ? ay : min_ay;
  max_ay = max_ay < ay ? ay : max_ay;
  min_az = min_az > az ? az : min_az;
  max_az = max_az < az ? az : max_az;

  min_gx = min_gx > gx ? gx : min_gx;
  max_gx = max_gx < gx ? gx : max_gx;
  min_gy = min_gy > gy ? gy : min_gy;
  max_gy = max_gy < gy ? gy : max_gy;
  min_gz = min_gz > gz ? gz : min_gz;
  max_gz = max_gz < gz ? gz : max_gz;

  Serial.print(min_ax); Serial.print(":"); Serial.print(max_ax); Serial.print("\t");
  Serial.print(min_ay); Serial.print(":"); Serial.print(max_ay); Serial.print("\t");
  Serial.print(min_az); Serial.print(":"); Serial.print(max_az); Serial.print("\t |");

  Serial.print(min_gx); Serial.print(":"); Serial.print(max_gx); Serial.print("\t");
  Serial.print(min_gy); Serial.print(":"); Serial.print(max_gy); Serial.print("\t");
  Serial.print(min_gz); Serial.print(":"); Serial.print(max_gz); Serial.print("\n\n");
  
  delay(300);
}
