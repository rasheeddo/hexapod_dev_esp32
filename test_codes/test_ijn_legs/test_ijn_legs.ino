#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define L1 60 //40
#define L2 80
#define L3 171 //163

#define RAD2DEG 180/PI
#define DEG2RAD PI/180

#define S -150   // starting point offset from 0
#define T 100    // step distance (how far the foot will move)
#define A 50     // step height (how the foot will lift from ground)
#define DATA_POINT_PER_LINE 50
#define DATA_POINT_ALL DATA_POINT_PER_LINE*2

#define pwm1_min 800.0
#define pwm1_mid 1350.0
#define pwm1_max 1900.0

#define pwm2_min 800.0
#define pwm2_mid 1500.0
#define pwm2_max 2200.0

#define pwm3_min 800.0
#define pwm3_mid 1500.0
#define pwm3_max 2200.0


double x = 150.0;
double y = 100.0;
double z = -100.0;

double x_start = 160.0;

double theta1;
double theta2;
double theta3;

double X1_line[DATA_POINT_PER_LINE];
double Y1_line[DATA_POINT_PER_LINE];
double Z1_line[DATA_POINT_PER_LINE];

double X2_line[DATA_POINT_PER_LINE];
double Y2_line[DATA_POINT_PER_LINE];
double Z2_line[DATA_POINT_PER_LINE];

double X6_line[DATA_POINT_PER_LINE];
double Y6_line[DATA_POINT_PER_LINE];
double Z6_line[DATA_POINT_PER_LINE];

double X1_curve[DATA_POINT_PER_LINE];
double Y1_curve[DATA_POINT_PER_LINE];
double Z1_curve[DATA_POINT_PER_LINE];

double X2_curve[DATA_POINT_PER_LINE];
double Y2_curve[DATA_POINT_PER_LINE];
double Z2_curve[DATA_POINT_PER_LINE];

double X6_curve[DATA_POINT_PER_LINE];
double Y6_curve[DATA_POINT_PER_LINE];
double Z6_curve[DATA_POINT_PER_LINE];

double X1[DATA_POINT_ALL];
double Y1[DATA_POINT_ALL];
double Z1[DATA_POINT_ALL];

double X2[DATA_POINT_ALL];
double Y2[DATA_POINT_ALL];
double Z2[DATA_POINT_ALL];

double X6[DATA_POINT_ALL];
double Y6[DATA_POINT_ALL];
double Z6[DATA_POINT_ALL];

double P1[2] = { -T / 2, S};
double P2[2] = {0, S + (2 * A)};
double P3[2] = {T / 2, S};
double t[DATA_POINT_PER_LINE];

double THETA1_1[DATA_POINT_ALL];
double THETA2_1[DATA_POINT_ALL];
double THETA3_1[DATA_POINT_ALL];

double THETA1_2[DATA_POINT_ALL];
double THETA2_2[DATA_POINT_ALL];
double THETA3_2[DATA_POINT_ALL];

double THETA1_6[DATA_POINT_ALL];
double THETA2_6[DATA_POINT_ALL];
double THETA3_6[DATA_POINT_ALL];

int PWM1_1[DATA_POINT_ALL];
int PWM2_1[DATA_POINT_ALL];
int PWM3_1[DATA_POINT_ALL];

int PWM1_2[DATA_POINT_ALL];
int PWM2_2[DATA_POINT_ALL];
int PWM3_2[DATA_POINT_ALL];

int PWM1_6[DATA_POINT_ALL];
int PWM2_6[DATA_POINT_ALL];
int PWM3_6[DATA_POINT_ALL];

double beta = 55.71;
double rot2_ang = -beta * DEG2RAD;
double rot6_ang = beta * DEG2RAD;

double R11_2 = cos(rot2_ang);
double R12_2 = -sin(rot2_ang);
double R13_2 = 0.0;
double R21_2 = sin(rot2_ang);
double R22_2 = cos(rot2_ang);
double R23_2 = 0.0;
double R31_2 = 0.0;
double R32_2 = 0.0;
double R33_2 = 1.0;

double R11_6 = cos(rot6_ang);
double R12_6 = -sin(rot6_ang);
double R13_6 = 0.0;
double R21_6 = sin(rot6_ang);
double R22_6 = cos(rot6_ang);
double R23_6 = 0.0;
double R31_6 = 0.0;
double R32_6 = 0.0;
double R33_6 = 1.0;

int counter = 0;
int j;

// sizeof() returns number of bytes not array's length

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm_2 = Adafruit_PWMServoDriver(0x41);

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

  /////////////////////
  /// Leg 1 (leg i) ///
  /////////////////////
  linspace(x_start, x_start, DATA_POINT_PER_LINE, X1_line);
  linspace(S, S, DATA_POINT_PER_LINE, Z1_line);

  linspace(T / 2, -T / 2, DATA_POINT_PER_LINE, Y1_line);

  // Create Bezier curve
  linspace(0.0, 1.0, DATA_POINT_PER_LINE, t);
  linspace(x_start, x_start, DATA_POINT_PER_LINE, X1_curve);
  for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
    Y1_curve[i] = (pow((1 - t[i]), 2) * P1[0]) + (2 * (1 - t[i]) * t[i] * P2[0]) + (pow(t[i], 2) * P3[0]);
    Z1_curve[i] = (pow((1 - t[i]), 2) * P1[1]) + (2 * (1 - t[i]) * t[i] * P2[1]) + (pow(t[i], 2) * P3[1]);
  }
  concat_arrays(X1_curve, X1_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, X1);
  concat_arrays(Y1_curve, Y1_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, Y1);
  concat_arrays(Z1_curve, Z1_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, Z1);

  inv_arrays(X1, Y1, Z1, DATA_POINT_ALL, THETA1_1, THETA2_1, THETA3_1);

  kinAngleArray_To_servoPwmArray(0, THETA1_1, DATA_POINT_ALL, PWM1_1);
  kinAngleArray_To_servoPwmArray(1, THETA2_1, DATA_POINT_ALL, PWM2_1);
  kinAngleArray_To_servoPwmArray(2, THETA3_1, DATA_POINT_ALL, PWM3_1);

  Serial.println("X1");
  print_array(X1, DATA_POINT_ALL);
  Serial.println("Y1");
  print_array(Y1, DATA_POINT_ALL);
  Serial.println("Z1");
  print_array(Z1, DATA_POINT_ALL);

  Serial.println("THETA1_1");
  print_array(THETA1_1, DATA_POINT_ALL);
  Serial.println("THETA2_1");
  print_array(THETA2_1, DATA_POINT_ALL);
  Serial.println("THETA3_1");
  print_array(THETA3_1, DATA_POINT_ALL);

  Serial.println("PWM1_1");
  print_array(PWM1_1, DATA_POINT_ALL);
  Serial.println("PWM2_1");
  print_array(PWM2_1, DATA_POINT_ALL);
  Serial.println("PWM3_1");
  print_array(PWM3_1, DATA_POINT_ALL);
  Serial.println("******************************************************");

  ////////////////////
  /// Leg2 (leg j) ///
  ///////////////////
  linspace(0.0, 0.0, DATA_POINT_PER_LINE, X2_line);
  linspace(S, S, DATA_POINT_PER_LINE, Z2_line);

  linspace(T / 2, -T / 2, DATA_POINT_PER_LINE, Y2_line);

  // Create Bezier curve
  linspace(0.0, 0.0, DATA_POINT_PER_LINE, X2_curve);
  for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
    Y2_curve[i] = (pow((1 - t[i]), 2) * P1[0]) + (2 * (1 - t[i]) * t[i] * P2[0]) + (pow(t[i], 2) * P3[0]);
    Z2_curve[i] = (pow((1 - t[i]), 2) * P1[1]) + (2 * (1 - t[i]) * t[i] * P2[1]) + (pow(t[i], 2) * P3[1]);
  }


  for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
    X2[i] = R11_2 * X2_line[i] + R12_2 * Y2_line[i] + R13_2 * Z2_line[i] + x_start;
    Y2[i] = R21_2 * X2_line[i] + R22_2 * Y2_line[i] + R23_2 * Z2_line[i];
    Z2[i] = R31_2 * X2_line[i] + R32_2 * Y2_line[i] + R33_2 * Z2_line[i];
  }

  j = 0;
  for (int i = DATA_POINT_PER_LINE; i < DATA_POINT_ALL; i++) {
    X2[i] = R11_2 * X2_curve[j] + R12_2 * Y2_curve[j] + R13_2 * Z2_curve[j] + x_start;
    Y2[i] = R21_2 * X2_curve[j] + R22_2 * Y2_curve[j] + R23_2 * Z2_curve[j];
    Z2[i] = R31_2 * X2_curve[j] + R32_2 * Y2_curve[j] + R33_2 * Z2_curve[j];
    j++;
  }

  inv_arrays(X2, Y2, Z2, DATA_POINT_ALL, THETA1_2, THETA2_2, THETA3_2);

  kinAngleArray_To_servoPwmArray(0, THETA1_2, DATA_POINT_ALL, PWM1_2);
  kinAngleArray_To_servoPwmArray(1, THETA2_2, DATA_POINT_ALL, PWM2_2);
  kinAngleArray_To_servoPwmArray(2, THETA3_2, DATA_POINT_ALL, PWM3_2);

  Serial.println("X2");
  print_array(X2, DATA_POINT_ALL);
  Serial.println("Y2");
  print_array(Y2, DATA_POINT_ALL);
  Serial.println("Z2");
  print_array(Z2, DATA_POINT_ALL);

  Serial.println("THETA1_2");
  print_array(THETA1_2, DATA_POINT_ALL);
  Serial.println("THETA2_2");
  print_array(THETA2_2, DATA_POINT_ALL);
  Serial.println("THETA3_2");
  print_array(THETA3_2, DATA_POINT_ALL);

  Serial.println("PWM1_2");
  print_array(PWM1_2, DATA_POINT_ALL);
  Serial.println("PWM2_2");
  print_array(PWM2_2, DATA_POINT_ALL);
  Serial.println("PWM3_2");
  print_array(PWM3_2, DATA_POINT_ALL);
  Serial.println("******************************************************");

  ////////////////////
  /// Leg6 (leg n) ///
  ///////////////////
  linspace(0.0, 0.0, DATA_POINT_PER_LINE, X6_line);
  linspace(S, S, DATA_POINT_PER_LINE, Z6_line);

  linspace(T / 2, -T / 2, DATA_POINT_PER_LINE, Y6_line);

  // Create Bezier curve
  linspace(0.0, 0.0, DATA_POINT_PER_LINE, X6_curve);
  for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
    Y6_curve[i] = (pow((1 - t[i]), 2) * P1[0]) + (2 * (1 - t[i]) * t[i] * P2[0]) + (pow(t[i], 2) * P3[0]);
    Z6_curve[i] = (pow((1 - t[i]), 2) * P1[1]) + (2 * (1 - t[i]) * t[i] * P2[1]) + (pow(t[i], 2) * P3[1]);
  }

  for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
    X6[i] = R11_6 * X6_line[i] + R12_6 * Y6_line[i] + R13_6 * Z6_line[i] + x_start;
    Y6[i] = R21_6 * X6_line[i] + R22_6 * Y6_line[i] + R23_6 * Z6_line[i];
    Z6[i] = R31_6 * X6_line[i] + R32_6 * Y6_line[i] + R33_6 * Z6_line[i];
  }

  j = 0;
  for (int i = DATA_POINT_PER_LINE; i < DATA_POINT_ALL; i++) {
    X6[i] = R11_6 * X6_curve[j] + R12_6 * Y6_curve[j] + R13_6 * Z6_curve[j] + x_start;
    Y6[i] = R21_6 * X6_curve[j] + R22_6 * Y6_curve[j] + R23_6 * Z6_curve[j];
    Z6[i] = R31_6 * X6_curve[j] + R32_6 * Y6_curve[j] + R33_6 * Z6_curve[j];
    j++;
  }

  inv_arrays(X6, Y6, Z6, DATA_POINT_ALL, THETA1_6, THETA2_6, THETA3_6);

  kinAngleArray_To_servoPwmArray(0, THETA1_6, DATA_POINT_ALL, PWM1_6);
  kinAngleArray_To_servoPwmArray(1, THETA2_6, DATA_POINT_ALL, PWM2_6);
  kinAngleArray_To_servoPwmArray(2, THETA3_6, DATA_POINT_ALL, PWM3_6);

  Serial.println("X6");
  print_array(X6, DATA_POINT_ALL);
  Serial.println("Y6");
  print_array(Y6, DATA_POINT_ALL);
  Serial.println("Z6");
  print_array(Z6, DATA_POINT_ALL);

  Serial.println("THETA1_6");
  print_array(THETA1_6, DATA_POINT_ALL);
  Serial.println("THETA2_6");
  print_array(THETA2_6, DATA_POINT_ALL);
  Serial.println("THETA3_6");
  print_array(THETA3_6, DATA_POINT_ALL);

  Serial.println("PWM1_6");
  print_array(PWM1_6, DATA_POINT_ALL);
  Serial.println("PWM2_6");
  print_array(PWM2_6, DATA_POINT_ALL);
  Serial.println("PWM3_6");
  print_array(PWM3_6, DATA_POINT_ALL);



  pwm.writeMicroseconds(0, int(pwm1_mid));
  pwm.writeMicroseconds(1, int(pwm2_mid));
  pwm.writeMicroseconds(2, int(pwm3_mid));

  pwm.writeMicroseconds(3, int(pwm1_mid));
  pwm.writeMicroseconds(4, int(pwm2_mid));
  pwm.writeMicroseconds(5, int(pwm3_mid));

  pwm.writeMicroseconds(15, int(pwm1_mid));
  pwm_2.writeMicroseconds(0, int(pwm2_mid));
  pwm_2.writeMicroseconds(1, int(pwm3_mid));

  delay(3000);




}

void loop() {

  if (counter < DATA_POINT_ALL) {

    pwm.writeMicroseconds(0, PWM1_1[counter]);
    pwm.writeMicroseconds(1, PWM2_1[counter]);
    pwm.writeMicroseconds(2, PWM3_1[counter]);

    pwm.writeMicroseconds(3, PWM1_2[counter]);
    pwm.writeMicroseconds(4, PWM2_2[counter]);
    pwm.writeMicroseconds(5, PWM3_2[counter]);

    pwm.writeMicroseconds(15, PWM1_6[counter]);
    pwm_2.writeMicroseconds(0, PWM2_6[counter]);
    pwm_2.writeMicroseconds(1, PWM3_6[counter]);


    counter++;

  } else {

    counter = 0;
  }

  delay(5);


}


void inv(double x, double y, double z, double& theta1, double& theta2, double& theta3) {
  theta1 = atan(y / x);
  double r2 = (x / cos(theta1)) - L1;
  double phi2 = atan(z / r2);
  double r1 = sqrt(pow(r2, 2) + pow(z, 2));
  double phi1 = acos(-( (pow(L3, 2) - pow(L2, 2) - pow(r1, 2)) / (2 * L2 * r1) ));

  theta2 = phi1 + phi2;

  double phi3 = acos(-( (pow(r1, 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3) ));

  theta3 = -(PI - phi3);
}

void linspace(double Start, double Stop, int Size, double* Arr) {

  double increment = (Stop - Start) / (Size - 1);

  for (int i = 0; i < Size ; i++) {

    if (i == 0) {
      Arr[i] = Start;
    } else {
      Arr[i] = Arr[i - 1] + increment;
    }

  }
}

void print_array(double* arr, int len) {

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

void concat_arrays(double* arr1, double* arr2, int arr1_len, int arr2_len, double* arr3) {

  for (int i = 0; i < arr1_len; i++) {
    arr3[i] = arr1[i];
  }

  for (int i = 0; i < arr2_len; i++) {
    arr3[arr1_len + i] = arr2[i];
  }
}

void inv_arrays(double* X, double* Y, double* Z, int len, double* THETA1, double* THETA2, double* THETA3) {

  for (int i = 0; i < len; i++) {
    double the1;
    double the2;
    double the3;

    inv(X[i], Y[i], Z[i], the1, the2, the3);

    THETA1[i] = the1 * RAD2DEG;
    THETA2[i] = the2 * RAD2DEG;
    THETA3[i] = the3 * RAD2DEG;
  }

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


void kinAngleArray_To_servoPwmArray(int servo_id, double* THETA, int len, int* PWM) {

  double pwm_val;

  for (int i = 0; i < len; i++) {
    //    if (servo_id == 0 || servo_id == 1) {
    //      pwm_val = map_with_limit(THETA[i], -70.0, 70.0, 800.0, 2200.0);
    //    } else {
    //      pwm_val = map_with_limit(THETA[i], -160.0, -20.0, 2200.0, 800.0);
    //    }

    if (servo_id == 0) {
      pwm_val = map_with_limit(THETA[i], -90.0, 90.0, pwm1_min, pwm1_max);
    } else if (servo_id == 1) {
      pwm_val = map_with_limit(THETA[i], -70.0, 70.0, pwm2_min, pwm2_max);
    } else {
      pwm_val = map_with_limit(THETA[i], -160.0, -20.0, pwm3_min, pwm3_max);
    }

    PWM[i] = int(pwm_val);

  }

}
