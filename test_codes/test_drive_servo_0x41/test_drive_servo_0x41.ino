#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 //50 // Analog servos run at ~50 Hz updates


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

void setup() {

  Serial.begin(112500);
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // Futaba A700
  //HIGH 1900
  //MID 1350
  //LOW 800
  // pwm.writeMicroseconds(0, 1350);
//  pwm.writeMicroseconds(1, 1500);
//  pwm.writeMicroseconds(2, 1400);

/// Futaba A700 CB700
//double kin_deg = 0.0;
//double servo_deg = map_with_limit(kin_deg, -90.0, 90.0, 0.0, 180.0);
//Serial.print("servo_deg ");
//Serial.println(servo_deg);
//int pwm_val = (int)map_with_limit(servo_deg, 0.0, 180.0, 800.0, 1900.0);
//Serial.print("pwm_val ");
//Serial.println(pwm_val);

/// Futaba DS3225
double kin_deg = 0.0;
double servo_deg = map_with_limit(kin_deg, -70.0, 70.0, 20.0, 160.0);
Serial.print("servo_deg ");
Serial.println(servo_deg);
int pwm_val = (int)map_with_limit(servo_deg, 20.0, 160.0, 800.0, 2200.0);
Serial.print("pwm_val ");
Serial.println(pwm_val);

pwm.writeMicroseconds(0, pwm_val);
pwm.writeMicroseconds(1, pwm_val);

}

void loop() {

//  pwm.writeMicroseconds(0, 800);
//
//  Serial.println("LOW");
//
//  delay(2000);
//
//  pwm.writeMicroseconds(0, 1350);
//
//  Serial.println("MID");
//
//  delay(2000);
//
//  pwm.writeMicroseconds(0, 1900);
//
//  Serial.println("HIGH");
//
//  delay(2000);

}

double map_with_limit(double val, double in_min, double in_max, double out_min, double out_max) {

  double m;
  double out;

  m = (out_max - out_min) / (in_max - in_min);
  out = m * (val - in_min) + out_min;

  if (out_min > out_max) {
    if (out > out_min) {
      out = out_min;
    }
    else if (out < out_max) {
      out = out_max;
    }

  }
  else if (out_max > out_min) {
    if (out > out_max) {
      out = out_max;
    }
    else if (out < out_min) {
      out = out_min;
    }
  }

  return out;

}
