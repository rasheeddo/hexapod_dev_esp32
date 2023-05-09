#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define RAD2DEG 180/PI
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

unsigned long period;
unsigned long last_stamp;
float qw;
float qx;
float qy;
float qz;
float r11, r12, r13, r21, r22, r23, r31, r32, r33;

float roll;
float pitch;
float yaw;

void setup() {
  Serial.begin(115200);

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  last_stamp = millis();
}

void loop() {

  period = millis() - last_stamp;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  sensors_event_t event;
  bno.getEvent(&event);

  imu::Quaternion quat = bno.getQuat();
  qw = quat.w();
  qx = quat.x();
  qy = quat.y();
  qz = quat.z();
  r11 = pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2); 
  r12 = 2*qx*qy - 2*qz*qw;
  r13 = 2*qx*qz + 2*qy*qw;
  r21 = 2*qx*qy + 2*qz*qw;
  r22 = pow(qw,2) - pow(qx,2) + pow(qy,2) - pow(qz,2); 
  r23 = 2*qy*qz - 2*qx*qw;
  r31 = 2*qx*qz - 2*qy*qw;
  r32 = 2*qy*qz + 2*qx*qw;
  r33 = pow(qw,2) - pow(qx,2) - pow(qy,2) + pow(qz,2); 

  roll = atan2(r32, r33) * RAD2DEG;
  pitch = asin(-r31) * RAD2DEG;
  yaw = atan2(-r21, r11) * RAD2DEG;

  /* Display the floating point data */
  Serial.print("period ");
  Serial.print(period, 6);
  Serial.print(" eul_X: ");
  Serial.print(euler.x());
  Serial.print(" eul_Y: ");
  Serial.print(euler.y());
  Serial.print(" eul_Z: ");
  Serial.print(euler.z());
  /* Display the floating point data */
  Serial.print("   ori_X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print(" ori_Y: ");
  Serial.print(event.orientation.y, 4);
  Serial.print(" ori_Z: ");
  Serial.print(event.orientation.z, 4);
  Serial.print("   qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("   roll: ");
  Serial.print(roll, 2);
  Serial.print("   pitch: ");
  Serial.print(pitch, 2);
  Serial.print("   yaw: ");
  Serial.print(yaw, 2);
  Serial.println("");

  // delay(1);

  last_stamp = millis();

}
