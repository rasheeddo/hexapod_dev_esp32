#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <hexapod_lib.h>

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm_2 = Adafruit_PWMServoDriver(0x41);

HexapodLib h;
int counter = 0;
int ang_inc = 0;

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(5);

  pwm_2.begin();
  pwm_2.setOscillatorFrequency(27000000);
  pwm_2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(5);

  Serial.println("******************************************************");
  Serial.print("PWM1_home ");
  Serial.println(h.PWM1_home);
  Serial.print("PWM2_home ");
  Serial.println(h.PWM2_home);
  Serial.print("PWM3_home ");
  Serial.println(h.PWM3_home);

//  Serial.println("******************************************************");
//
//  Serial.println("X3");
//  print_array(h.X3, DATA_POINT_ALL);
//  Serial.println("Y3");
//  print_array(h.Y3, DATA_POINT_ALL);
//  Serial.println("Z3");
//  print_array(h.Z3, DATA_POINT_ALL);
//
//  Serial.println("******************************************************");
//
//  Serial.println("X4");
//  print_array(h.X4, DATA_POINT_ALL);
//  Serial.println("Y4");
//  print_array(h.Y4, DATA_POINT_ALL);
//  Serial.println("Z4");
//  print_array(h.Z4, DATA_POINT_ALL);

  pwm.writeMicroseconds(0, h.PWM1_home);
  pwm.writeMicroseconds(1, h.PWM2_home);
  pwm.writeMicroseconds(2, h.PWM3_home);

  pwm.writeMicroseconds(3, h.PWM1_home);
  pwm.writeMicroseconds(4, h.PWM2_home);
  pwm.writeMicroseconds(5, h.PWM3_home);

  pwm.writeMicroseconds(6, h.PWM1_home);
  pwm.writeMicroseconds(7, h.PWM2_home);
  pwm.writeMicroseconds(8, h.PWM3_home);

  pwm.writeMicroseconds(9, h.PWM1_home);
  pwm.writeMicroseconds(10, h.PWM2_home);
  pwm.writeMicroseconds(11, h.PWM3_home);

  pwm.writeMicroseconds(12, h.PWM1_home);
  pwm.writeMicroseconds(13, h.PWM2_home);
  pwm.writeMicroseconds(14, h.PWM3_home);

  pwm.writeMicroseconds(15, h.PWM1_home);
  pwm_2.writeMicroseconds(0, h.PWM2_home);
  pwm_2.writeMicroseconds(1, h.PWM3_home);

  int pwm1_mid = (int)h.pwm1_mid;
  int pwm2_mid = (int)h.pwm2_mid;
  int pwm3_mid = (int)h.pwm3_mid;

  //  pwm.writeMicroseconds(0, pwm1_mid);
  //  pwm.writeMicroseconds(1, pwm2_mid);
  //  pwm.writeMicroseconds(2, pwm3_mid);
  //
  //  pwm.writeMicroseconds(3, pwm1_mid);
  //  pwm.writeMicroseconds(4, pwm2_mid);
  //  pwm.writeMicroseconds(5, pwm3_mid);
  //
  //  pwm.writeMicroseconds(6, pwm1_mid);
  //  pwm.writeMicroseconds(7, pwm2_mid);
  //  pwm.writeMicroseconds(8, pwm3_mid);
  //
  //  pwm.writeMicroseconds(9, pwm1_mid);
  //  pwm.writeMicroseconds(10, pwm2_mid);
  //  pwm.writeMicroseconds(11, pwm3_mid);
  //
  //  pwm.writeMicroseconds(12, pwm1_mid);
  //  pwm.writeMicroseconds(13, pwm2_mid);
  //  pwm.writeMicroseconds(14, pwm3_mid);
  //
  //  pwm.writeMicroseconds(15, pwm1_mid);
  //  pwm_2.writeMicroseconds(0, pwm2_mid);
  //  pwm_2.writeMicroseconds(1, pwm3_mid);

  delay(3000);
}

void loop() {
  if (counter < DATA_POINT_ALL) {

    pwm.writeMicroseconds(0, h.crab_walking_LUT[ang_inc][0][counter]);
    pwm.writeMicroseconds(1, h.crab_walking_LUT[ang_inc][1][counter]);
    pwm.writeMicroseconds(2, h.crab_walking_LUT[ang_inc][2][counter]);

    pwm.writeMicroseconds(3, h.crab_walking_LUT[ang_inc][3][counter]);
    pwm.writeMicroseconds(4, h.crab_walking_LUT[ang_inc][4][counter]);
    pwm.writeMicroseconds(5, h.crab_walking_LUT[ang_inc][5][counter]);

    pwm.writeMicroseconds(6, h.crab_walking_LUT[ang_inc][6][counter]);
    pwm.writeMicroseconds(7, h.crab_walking_LUT[ang_inc][7][counter]);
    pwm.writeMicroseconds(8, h.crab_walking_LUT[ang_inc][8][counter]);

    pwm.writeMicroseconds(9, h.crab_walking_LUT[ang_inc][9][counter]);
    pwm.writeMicroseconds(10, h.crab_walking_LUT[ang_inc][10][counter]);
    pwm.writeMicroseconds(11, h.crab_walking_LUT[ang_inc][11][counter]);

    pwm.writeMicroseconds(12, h.crab_walking_LUT[ang_inc][12][counter]);
    pwm.writeMicroseconds(13, h.crab_walking_LUT[ang_inc][13][counter]);
    pwm.writeMicroseconds(14, h.crab_walking_LUT[ang_inc][14][counter]);

    pwm.writeMicroseconds(15, h.crab_walking_LUT[ang_inc][15][counter]);
    pwm_2.writeMicroseconds(0, h.crab_walking_LUT[ang_inc][16][counter]);
    pwm_2.writeMicroseconds(1, h.crab_walking_LUT[ang_inc][17][counter]);


    counter++;

  } else {

    counter = 0;
  }

  delay(10);

}

void print_array(double* arr, int len) {

  for (int i = 0; i < len; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println(" ");

}


void print_array(float* arr, int len) {

  for (int i = 0; i < len; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println(" ");

}

void print_array(int* arr, int len) {

  for (int i = 0; i < len; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println(" ");

}
