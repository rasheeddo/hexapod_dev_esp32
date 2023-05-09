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

  //  Serial.println("X1");
  //  print_array(h.x1, DATA_POINT_ALL);
  //  Serial.println("Y1");
  //  print_array(h.y1, DATA_POINT_ALL);
  //  Serial.println("Z1");
  //  print_array(h.z1, DATA_POINT_ALL);
  //
  //  Serial.println("THETA1_1");
  //  print_array(h.the1_1, DATA_POINT_ALL);
  //  Serial.println("THETA2_1");
  //  print_array(h.the2_1, DATA_POINT_ALL);
  //  Serial.println("THETA3_1");
  //  print_array(h.the3_1, DATA_POINT_ALL);
  //
  //  Serial.println("PWM1_1");
  //  print_array(h.pwm1_1, DATA_POINT_ALL);
  //  Serial.println("PWM2_1");
  //  print_array(h.pwm2_1, DATA_POINT_ALL);
  //  Serial.println("PWM3_1");
  //  print_array(h.pwm3_1, DATA_POINT_ALL);
  //  Serial.println("******************************************************");
  //
  //  Serial.println("X2");
  //  print_array(h.X2, DATA_POINT_ALL);
  //  Serial.println("Y2");
  //  print_array(h.Y2, DATA_POINT_ALL);
  //  Serial.println("Z2");
  //  print_array(h.Z2, DATA_POINT_ALL);
  //
  //  Serial.println("THETA1_2");
  //  print_array(h.THETA1_2, DATA_POINT_ALL);
  //  Serial.println("THETA2_2");
  //  print_array(h.THETA2_2, DATA_POINT_ALL);
  //  Serial.println("THETA3_2");
  //  print_array(h.THETA3_2, DATA_POINT_ALL);
  //
  //  Serial.println("PWM1_2");
  //  print_array(h.PWM1_2, DATA_POINT_ALL);
  //  Serial.println("PWM2_2");
  //  print_array(h.PWM2_2, DATA_POINT_ALL);
  //  Serial.println("PWM3_2");
  //  print_array(h.PWM3_2, DATA_POINT_ALL);
  //  Serial.println("******************************************************");
  //
  //  Serial.println("X6");
  //  print_array(h.X6, DATA_POINT_ALL);
  //  Serial.println("Y6");
  //  print_array(h.Y6, DATA_POINT_ALL);
  //  Serial.println("Z6");
  //  print_array(h.Z6, DATA_POINT_ALL);
  //
  //  Serial.println("THETA1_6");
  //  print_array(h.THETA1_6, DATA_POINT_ALL);
  //  Serial.println("THETA2_6");
  //  print_array(h.THETA2_6, DATA_POINT_ALL);
  //  Serial.println("THETA3_6");
  //  print_array(h.THETA3_6, DATA_POINT_ALL);
  //
  //  Serial.println("PWM1_6");
  //  print_array(h.PWM1_6, DATA_POINT_ALL);
  //  Serial.println("PWM2_6");
  //  print_array(h.PWM2_6, DATA_POINT_ALL);
  //  Serial.println("PWM3_6");
  //  print_array(h.PWM3_6, DATA_POINT_ALL);

  Serial.println("******************************************************");
  Serial.print("PWM1_home ");
  Serial.println(h.PWM1_home);
  Serial.print("PWM2_home ");
  Serial.println(h.PWM2_home);
  Serial.print("PWM3_home ");
  Serial.println(h.PWM3_home);

  Serial.println("******************************************************");
  //  h.generate_crabWalkingLUT();
  for (int i = 0; i < 12; i++) {
    Serial.print("Ang ");
    Serial.println(i*30);
    Serial.println("CW PWM1_1");
    print_array(h.crab_walking_LUT[i][0], DATA_POINT_ALL);
    Serial.println("CW PWM2_1");
    print_array(h.crab_walking_LUT[i][1], DATA_POINT_ALL);
    Serial.println("CW PWM3_1");
    print_array(h.crab_walking_LUT[i][2], DATA_POINT_ALL);
    Serial.println("======================================");
    Serial.println("CW PWM1_2");
    print_array(h.crab_walking_LUT[i][3], DATA_POINT_ALL);
    Serial.println("CW PWM2_2");
    print_array(h.crab_walking_LUT[i][4], DATA_POINT_ALL);
    Serial.println("CW PWM3_2");
    print_array(h.crab_walking_LUT[i][5], DATA_POINT_ALL);
    Serial.println("======================================");
    Serial.println("CW PWM1_3");
    print_array(h.crab_walking_LUT[i][6], DATA_POINT_ALL);
    Serial.println("CW PWM2_3");
    print_array(h.crab_walking_LUT[i][7], DATA_POINT_ALL);
    Serial.println("CW PWM3_3");
    print_array(h.crab_walking_LUT[i][8], DATA_POINT_ALL);
    Serial.println("======================================");
    Serial.println("CW PWM1_4");
    print_array(h.crab_walking_LUT[i][9], DATA_POINT_ALL);
    Serial.println("CW PWM2_4");
    print_array(h.crab_walking_LUT[i][10], DATA_POINT_ALL);
    Serial.println("CW PWM3_4");
    print_array(h.crab_walking_LUT[i][11], DATA_POINT_ALL);
    Serial.println("======================================");
    Serial.println("CW PWM1_5");
    print_array(h.crab_walking_LUT[i][12], DATA_POINT_ALL);
    Serial.println("CW PWM2_5");
    print_array(h.crab_walking_LUT[i][13], DATA_POINT_ALL);
    Serial.println("CW PWM3_5");
    print_array(h.crab_walking_LUT[i][14], DATA_POINT_ALL);
    Serial.println("======================================");
    Serial.println("CW PWM1_6");
    print_array(h.crab_walking_LUT[i][15], DATA_POINT_ALL);
    Serial.println("CW PWM2_6");
    print_array(h.crab_walking_LUT[i][16], DATA_POINT_ALL);
    Serial.println("CW PWM3_6");
    print_array(h.crab_walking_LUT[i][17], DATA_POINT_ALL);

    Serial.println("******************************************************");
  }

//    Serial.println("X3");
//    print_array(h.X3, DATA_POINT_ALL);
//    Serial.println("Y3");
//    print_array(h.Y3, DATA_POINT_ALL);
//    Serial.println("Z3");
//    print_array(h.Z3, DATA_POINT_ALL);
//  
//    Serial.println("THETA1_3");
//    print_array(h.THETA1_3, DATA_POINT_ALL);
//    Serial.println("THETA2_3");
//    print_array(h.THETA2_3, DATA_POINT_ALL);
//    Serial.println("THETA3_3");
//    print_array(h.THETA3_3, DATA_POINT_ALL);
//    
//    Serial.println("******************************************************");





  pwm.writeMicroseconds(0, h.PWM1_home);
  pwm.writeMicroseconds(1, h.PWM2_home);
  pwm.writeMicroseconds(2, h.PWM3_home);

  pwm.writeMicroseconds(3, h.PWM1_home);
  pwm.writeMicroseconds(4, h.PWM2_home);
  pwm.writeMicroseconds(5, h.PWM3_home);

  pwm.writeMicroseconds(15, h.PWM1_home);
  pwm_2.writeMicroseconds(0, h.PWM2_home);
  pwm_2.writeMicroseconds(1, h.PWM3_home);

  delay(3000);
}

void loop() {
  //  if (counter < DATA_POINT_ALL) {
  //
  //    pwm.writeMicroseconds(0, h.PWM1_1[counter]);
  //    pwm.writeMicroseconds(1, h.PWM2_1[counter]);
  //    pwm.writeMicroseconds(2, h.PWM3_1[counter]);
  //
  //    pwm.writeMicroseconds(3, h.PWM1_2[counter]);
  //    pwm.writeMicroseconds(4, h.PWM2_2[counter]);
  //    pwm.writeMicroseconds(5, h.PWM3_2[counter]);
  //
  //    pwm.writeMicroseconds(15, h.PWM1_6[counter]);
  //    pwm_2.writeMicroseconds(0, h.PWM2_6[counter]);
  //    pwm_2.writeMicroseconds(1, h.PWM3_6[counter]);
  //
  //
  //    counter++;
  //
  //  } else {
  //
  //    counter = 0;
  //  }
  //
  //  delay(10);

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

//void print_cw_LUT(double* arr) {
//  for (int i = 0; i < 12; i++) {
//    for (int j = 0; j < 18; j++) {
//      for (int k=0; k < 40; j++){
//
//      }
//    }
//  }
//}
