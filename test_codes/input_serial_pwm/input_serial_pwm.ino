#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 //50 // Analog servos run at ~50 Hz updates

int input;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {

  Serial.begin(112500);
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

}

void loop() {

  if (Serial.available() > 0){

    input = Serial.read();

    Serial.println(input);
    
  }



}
