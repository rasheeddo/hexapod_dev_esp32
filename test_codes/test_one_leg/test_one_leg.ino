#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define L1 60 //40
#define L2 80
#define L3 171 //163

#define RAD2DEG 180/PI

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

double X_line[DATA_POINT_PER_LINE];
double Y_line[DATA_POINT_PER_LINE];
double Z_line[DATA_POINT_PER_LINE];

double X_curve[DATA_POINT_PER_LINE];
double Y_curve[DATA_POINT_PER_LINE];
double Z_curve[DATA_POINT_PER_LINE];

double X[DATA_POINT_ALL];
double Y[DATA_POINT_ALL];
double Z[DATA_POINT_ALL];

double P1[2] = { -T / 2, S};
double P2[2] = {0, S + (2 * A)};
double P3[2] = {T / 2, S};
double t[DATA_POINT_PER_LINE];

double THETA1[DATA_POINT_ALL];
double THETA2[DATA_POINT_ALL];
double THETA3[DATA_POINT_ALL];

int PWM1[DATA_POINT_ALL];
int PWM2[DATA_POINT_ALL];
int PWM3[DATA_POINT_ALL];

int counter = 0;

// sizeof() returns number of bytes not array's length

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {

  Serial.begin(115200);
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(5);

  linspace(x_start, x_start, DATA_POINT_PER_LINE, X_line);
  linspace(S, S, DATA_POINT_PER_LINE, Z_line);

  //inv(x, y, z, theta1, theta2, theta3);

  linspace(T / 2, -T / 2, DATA_POINT_PER_LINE, Y_line);

  // Create Bezier curve
  linspace(0.0, 1.0, DATA_POINT_PER_LINE, t);
  linspace(x_start, x_start, DATA_POINT_PER_LINE, X_curve);
  for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
    Y_curve[i] = (pow((1 - t[i]), 2) * P1[0]) + (2 * (1 - t[i]) * t[i] * P2[0]) + (pow(t[i], 2) * P3[0]);
    Z_curve[i] = (pow((1 - t[i]), 2) * P1[1]) + (2 * (1 - t[i]) * t[i] * P2[1]) + (pow(t[i], 2) * P3[1]);
  }


  concat_arrays(X_curve, X_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, X);
  concat_arrays(Y_curve, Y_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, Y);
  concat_arrays(Z_curve, Z_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, Z);

  inv_arrays(X, Y, Z, DATA_POINT_ALL, THETA1, THETA2, THETA3);

  kinAngleArray_To_servoPwmArray(0, THETA1, DATA_POINT_ALL, PWM1);
  kinAngleArray_To_servoPwmArray(1, THETA2, DATA_POINT_ALL, PWM2);
  kinAngleArray_To_servoPwmArray(2, THETA3, DATA_POINT_ALL, PWM3);

  Serial.println("t");
  print_array(t, DATA_POINT_PER_LINE);
  Serial.println("X_line");
  print_array(X_line, DATA_POINT_PER_LINE);
  Serial.println("Y_line");
  print_array(Y_line, DATA_POINT_PER_LINE);
  Serial.println("Z_line");
  print_array(Z_line, DATA_POINT_PER_LINE);
  Serial.println("X_curve");
  print_array(X_curve, DATA_POINT_PER_LINE);
  Serial.println("Y_curve");
  print_array(Y_curve, DATA_POINT_PER_LINE);
  Serial.println("Z_curve");
  print_array(Z_curve, DATA_POINT_PER_LINE);
  Serial.println("X");
  print_array(X, DATA_POINT_ALL);
  Serial.println("Y");
  print_array(Y, DATA_POINT_ALL);
  Serial.println("Z");
  print_array(Z, DATA_POINT_ALL);

  Serial.println("THETA1");
  print_array(THETA1, DATA_POINT_ALL);
  Serial.println("THETA2");
  print_array(THETA2, DATA_POINT_ALL);
  Serial.println("THETA3");
  print_array(THETA3, DATA_POINT_ALL);

  Serial.println("PWM1");
  print_array(PWM1, DATA_POINT_ALL);
  Serial.println("PWM2");
  print_array(PWM2, DATA_POINT_ALL);
  Serial.println("PWM3");
  print_array(PWM3, DATA_POINT_ALL);

  pwm.writeMicroseconds(0, int(pwm1_mid));
  pwm.writeMicroseconds(1, int(pwm2_mid));
  pwm.writeMicroseconds(2, int(pwm3_mid));

  delay(3000);




}

void loop() {

  if (counter < DATA_POINT_ALL) {

    pwm.writeMicroseconds(0, PWM1[counter]);
//    pwm.writeMicroseconds(0, 1350);/
    pwm.writeMicroseconds(1, PWM2[counter]);
    pwm.writeMicroseconds(2, PWM3[counter]);

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
