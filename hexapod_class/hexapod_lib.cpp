#include "Arduino.h"
#include "hexapod_lib.h"

HexapodLib::HexapodLib(){

	inv(X_home, Y_home, Z_home, theta1_home, theta2_home, theta3_home);


	PWM1_home = kinAngle_To_servoPwm(0, theta1_home*RAD2DEG);    // 0.0
	PWM2_home = kinAngle_To_servoPwm(1, theta2_home*RAD2DEG);    // 14.13
	PWM3_home = kinAngle_To_servoPwm(2, theta3_home*RAD2DEG);    // -96/59

}
void HexapodLib::fwd(float theta1, float theta2, float theta3, float& x, float& y, float& z){
		x = L1*cos(theta1) + (L2*cos(theta2) + L3*cos(theta2+theta3))*cos(theta1);
		y = L1*sin(theta1) + (L2*cos(theta2) + L3*cos(theta2+theta3))*sin(theta1);
		z = L2*sin(theta2) + L3*sin(theta2+theta3);
}
void HexapodLib::inv(float x, float y, float z, float& theta1, float& theta2, float& theta3) {

  theta1 = atan(y / x);
  float r2 = (x / cos(theta1)) - L1;
  float phi2 = atan(z / r2);
  float r1 = sqrt(pow(r2, 2) + pow(z, 2));
  float phi1 = acos(-( (pow(L3, 2) - pow(L2, 2) - pow(r1, 2)) / (2 * L2 * r1) ));

  theta2 = phi1 + phi2;

  float phi3 = acos(-( (pow(r1, 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3) ));

  theta3 = -(PI - phi3);
}

void HexapodLib::linspace(float Start, float Stop, int _size, float* Arr) {

  float increment = (Stop - Start) / (_size - 1);

  for (int i = 0; i < _size ; i++) {

	if (i == 0) {
	  Arr[i] = Start;
	} else {
	  Arr[i] = Arr[i - 1] + increment;
	}

  }
}

void HexapodLib::concat_arrays(float* arr1, float* arr2, int arr1_len, int arr2_len, float* arr3) {

  for (int i = 0; i < arr1_len; i++) {
	arr3[i] = arr1[i];
  }

  for (int i = 0; i < arr2_len; i++) {
	arr3[arr1_len + i] = arr2[i];
  }
}

void HexapodLib::inv_arrays(float* x_arr, float* y_arr, float* z_arr, int len, float* THE1, float* THE2, float* THE3) {

  for (int i = 0; i < len; i++) {
	float the1;
	float the2;
	float the3;

	inv(x_arr[i], y_arr[i], z_arr[i], the1, the2, the3);

	THE1[i] = the1 * RAD2DEG;
	THE2[i] = the2 * RAD2DEG;
	THE3[i] = the3 * RAD2DEG;
  }

}

void HexapodLib::inv_arrays_RAD(float* x_arr, float* y_arr, float* z_arr, int len, float* THE1, float* THE2, float* THE3) {

  for (int i = 0; i < len; i++) {
		float the1;
		float the2;
		float the3;

		inv(x_arr[i], y_arr[i], z_arr[i], the1, the2, the3);

		THE1[i] = the1;
		THE2[i] = the2;
		THE3[i] = the3;
  }

}

void HexapodLib::invKinArray_to_ThetaArray(float XYZ[DATA_POINT_ALL][3], float* THE1, float* THE2, float* THE3){

	for (int i = 0; i < DATA_POINT_ALL; i++) {
		float the1;
		float the2;
		float the3;

		inv(XYZ[i][0], XYZ[i][1], XYZ[i][2], the1, the2, the3);

		THE1[i] = the1;
		THE2[i] = the2;
		THE3[i] = the3;
  }
}

void HexapodLib::invKinArray_to_ThetaDegArray(float XYZ[DATA_POINT_ALL][3], float* THE1, float* THE2, float* THE3){

	for (int i = 0; i < DATA_POINT_ALL; i++) {
		float the1;
		float the2;
		float the3;

		inv(XYZ[i][0], XYZ[i][1], XYZ[i][2], the1, the2, the3);

		THE1[i] = the1*RAD2DEG;
		THE2[i] = the2*RAD2DEG;
		THE3[i] = the3*RAD2DEG;
  }
}

float HexapodLib::map_with_limit(float val, float in_min, float in_max, float out_min, float out_max) {

  float m;
  float out;

  m = (out_max - out_min) / (in_max - in_min);
  out = m * (val - in_min) + out_min;

	if (out_min > out_max) {
		if (out > out_min) {
			out = out_min;
		} else if (out < out_max) {
			out = out_max;
		}
	} else if (out_max > out_min) {
		if (out > out_max) {
			out = out_max;
		}
		else if (out < out_min) {
			out = out_min;
		}
	}

  return out;

}

int HexapodLib::kinAngle_To_servoPwm(int servo_id, float theta){
	float pwm_val;
	if (servo_id == 0) {
		pwm_val = map_with_limit(theta, joint1_min, joint1_max, pwm1_min, pwm1_max);
	} else if (servo_id == 1) {
		pwm_val = map_with_limit(theta, joint2_min, joint2_max, pwm2_min, pwm2_max);
	} else {
		pwm_val = map_with_limit(theta, joint3_min, joint3_max, pwm3_min, pwm3_max);
	}
	return (int) pwm_val;
}

void HexapodLib::kinAngleArray_To_servoPwmArray(int servo_id, float* THETA, int len, int* PWM) {

  float pwm_val;

  for (int i = 0; i < len; i++) {
	//    if (servo_id == 0 || servo_id == 1) {
	//      pwm_val = map_with_limit(THETA[i], -70.0, 70.0, 800.0, 2200.0);
	//    } else {
	//      pwm_val = map_with_limit(THETA[i], -160.0, -20.0, 2200.0, 800.0);
	//    }

	if (servo_id == 0) {
		pwm_val = map_with_limit(THETA[i], joint1_min, joint1_max, pwm1_min, pwm1_max);
	} else if (servo_id == 1) {
		pwm_val = map_with_limit(THETA[i], joint2_min, joint2_max, pwm2_min, pwm2_max);
	} else {
		pwm_val = map_with_limit(THETA[i], joint3_min, joint3_max, pwm3_min, pwm3_max);
	}

	PWM[i] = (int) pwm_val;

  }

}

void HexapodLib::XYZ_gen(int leg_id, float dir_rot_ang, float* x_arr, float* y_arr, float* z_arr){
	
	float rot;
	int j;

	float X_line[DATA_POINT_PER_LINE];
	float Y_line[DATA_POINT_PER_LINE];
	float Z_line[DATA_POINT_PER_LINE];
	float X_curve[DATA_POINT_PER_LINE];
	float Y_curve[DATA_POINT_PER_LINE];
	float Z_curve[DATA_POINT_PER_LINE];

	float P1[2] = { -T / 2, S};
	float P2[2] = {0, S + (2 * A)};
	float P3[2] = {T / 2, S};
	float t[DATA_POINT_PER_LINE];


	if (leg_id == 1){
		rot = walk_rot_ang1 + (dir_rot_ang*DEG2RAD);
	} else if (leg_id == 2){
		rot = walk_rot_ang2 + (dir_rot_ang*DEG2RAD);
	} else if (leg_id == 3){
		rot = walk_rot_ang3 + (dir_rot_ang*DEG2RAD);
	} else if (leg_id == 4){
		rot = walk_rot_ang4 + (dir_rot_ang*DEG2RAD);
	} else if (leg_id == 5){
		rot = walk_rot_ang5 + (dir_rot_ang*DEG2RAD);
	} else if (leg_id == 6){
		rot = walk_rot_ang6 + (dir_rot_ang*DEG2RAD);
	}


	// rotation matrix
	float R11 = cos(rot);
	float R12 = -sin(rot);
	float R13 = 0.0;
	float R21 = sin(rot);
	float R22 = cos(rot);
	float R23 = 0.0;
	float R31 = 0.0;
	float R32 = 0.0;
	float R33 = 1.0;

	linspace(0.0, 1.0, DATA_POINT_PER_LINE, t);
	linspace(S, S, DATA_POINT_PER_LINE, Z_line);
	linspace(0.0, 0.0, DATA_POINT_PER_LINE, X_line);
	linspace(0.0, 0.0, DATA_POINT_PER_LINE, X_curve);

	if (leg_id == 1) {

		/// Generate Line and curve path of foot
		
		linspace(T / 2, -T / 2, DATA_POINT_PER_LINE, Y_line);
		
		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			Y_curve[i] = (pow((1 - t[i]), 2) * P1[0]) + (2 * (1 - t[i]) * t[i] * P2[0]) + (pow(t[i], 2) * P3[0]);
			Z_curve[i] = (pow((1 - t[i]), 2) * P1[1]) + (2 * (1 - t[i]) * t[i] * P2[1]) + (pow(t[i], 2) * P3[1]);
		}

		/// Curve first then Line
		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			x_arr[i] = (R11 * X_curve[i]) + (R12 * Y_curve[i]) + (R13 * Z_curve[i]) + x_start;
			y_arr[i] = (R21 * X_curve[i]) + (R22 * Y_curve[i]) + (R23 * Z_curve[i]);
			z_arr[i] = (R31 * X_curve[i]) + (R32 * Y_curve[i]) + (R33 * Z_curve[i]);
		}
		j = 0;
		for (int i = DATA_POINT_PER_LINE; i < DATA_POINT_ALL; i++) {
			x_arr[i] = (R11 * X_line[j]) + (R12 * Y_line[j]) + (R13 * Z_line[j]) + x_start;
			y_arr[i] = (R21 * X_line[j]) + (R22 * Y_line[j]) + (R23 * Z_line[j]);
			z_arr[i] = (R31 * X_line[j]) + (R32 * Y_line[j]) + (R33 * Z_line[j]);
			j++;
		}


	} else if ((leg_id == 2) || (leg_id == 6)){

		/// Generate Line and curve path of foot
		// linspace(0.0, 0.0, DATA_POINT_PER_LINE, X_line);
		// linspace(0.0, 0.0, DATA_POINT_PER_LINE, X_curve);
		// linspace(S, S, DATA_POINT_PER_LINE, Z_line);
		linspace(T / 2, -T / 2, DATA_POINT_PER_LINE, Y_line);
		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			Y_curve[i] = (pow((1 - t[i]), 2) * P1[0]) + (2 * (1 - t[i]) * t[i] * P2[0]) + (pow(t[i], 2) * P3[0]);
			Z_curve[i] = (pow((1 - t[i]), 2) * P1[1]) + (2 * (1 - t[i]) * t[i] * P2[1]) + (pow(t[i], 2) * P3[1]);
		}


		/// Line first then Curve
		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			x_arr[i] = (R11 * X_line[i]) + (R12 * Y_line[i]) + (R13 * Z_line[i]) + x_start;
			y_arr[i] = (R21 * X_line[i]) + (R22 * Y_line[i]) + (R23 * Z_line[i]);
			z_arr[i] = (R31 * X_line[i]) + (R32 * Y_line[i]) + (R33 * Z_line[i]);
		}
		j = 0;
		for (int i = DATA_POINT_PER_LINE; i < DATA_POINT_ALL; i++) {
			x_arr[i] = (R11 * X_curve[j]) + (R12 * Y_curve[j]) + (R13 * Z_curve[j]) + x_start;
			y_arr[i] = (R21 * X_curve[j]) + (R22 * Y_curve[j]) + (R23 * Z_curve[j]);
			z_arr[i] = (R31 * X_curve[j]) + (R32 * Y_curve[j]) + (R33 * Z_curve[j]);
			j++;
		}

  	} else if ((leg_id == 3) || (leg_id == 5)){

  		/// Generate Line and curve path of foot
  		// curve will start from + to -
  		// line will start from - to +
  		// it's like a flip of 1st leg
		// linspace(0.0, 0.0, DATA_POINT_PER_LINE, X_line);
		// linspace(0.0, 0.0, DATA_POINT_PER_LINE, X_curve);
		// linspace(S, S, DATA_POINT_PER_LINE, Z_line);
		linspace(-T / 2, T / 2, DATA_POINT_PER_LINE, Y_line);
		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			Y_curve[DATA_POINT_PER_LINE-1-i] = (pow((1 - t[i]), 2) * P1[0]) + (2 * (1 - t[i]) * t[i] * P2[0]) + (pow(t[i], 2) * P3[0]);
			Z_curve[DATA_POINT_PER_LINE-1-i] = (pow((1 - t[i]), 2) * P1[1]) + (2 * (1 - t[i]) * t[i] * P2[1]) + (pow(t[i], 2) * P3[1]);
		}

		/// Curve first then Line
		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			x_arr[i] = (R11 * X_curve[i]) + (R12 * Y_curve[i]) + (R13 * Z_curve[i]) + x_start;
			y_arr[i] = (R21 * X_curve[i]) + (R22 * Y_curve[i]) + (R23 * Z_curve[i]);
			z_arr[i] = (R31 * X_curve[i]) + (R32 * Y_curve[i]) + (R33 * Z_curve[i]);
		}
		j = 0;
		for (int i = DATA_POINT_PER_LINE; i < DATA_POINT_ALL; i++) {
			x_arr[i] = (R11 * X_line[j]) + (R12 * Y_line[j]) + (R13 * Z_line[j]) + x_start;
			y_arr[i] = (R21 * X_line[j]) + (R22 * Y_line[j]) + (R23 * Z_line[j]);
			z_arr[i] = (R31 * X_line[j]) + (R32 * Y_line[j]) + (R33 * Z_line[j]);
			j++;
		}

  	} else if (leg_id == 4){

  		linspace(T / 2, -T / 2, DATA_POINT_PER_LINE, Y_line);
		
		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			Y_curve[i] = (pow((1 - t[i]), 2) * P1[0]) + (2 * (1 - t[i]) * t[i] * P2[0]) + (pow(t[i], 2) * P3[0]);
			Z_curve[DATA_POINT_PER_LINE-1-i] = (pow((1 - t[i]), 2) * P1[1]) + (2 * (1 - t[i]) * t[i] * P2[1]) + (pow(t[i], 2) * P3[1]);
		}

  		/// Line first then Curve
		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			x_arr[i] = (R11 * X_line[i]) + (R12 * Y_line[i]) + (R13 * Z_line[i]) + x_start;
			y_arr[i] = (R21 * X_line[i]) + (R22 * Y_line[i]) + (R23 * Z_line[i]);
			z_arr[i] = (R31 * X_line[i]) + (R32 * Y_line[i]) + (R33 * Z_line[i]);
		}
		j = 0;
		for (int i = DATA_POINT_PER_LINE; i < DATA_POINT_ALL; i++) {
			x_arr[i] = (R11 * X_curve[j]) + (R12 * Y_curve[j]) + (R13 * Z_curve[j]) + x_start;
			y_arr[i] = (R21 * X_curve[j]) + (R22 * Y_curve[j]) + (R23 * Z_curve[j]);
			z_arr[i] = (R31 * X_curve[j]) + (R32 * Y_curve[j]) + (R33 * Z_curve[j]);
			j++;
		}
  	} 
  
}

void HexapodLib::generate_crabWalkingLUT(){

	// Tried using global variables in class as X,Y,Z, THETA1, THETA2, THETA3, ...
	// but the value in array after XYZ_gen got wrong...

	float x_arr[DATA_POINT_ALL];
	float y_arr[DATA_POINT_ALL];
	float z_arr[DATA_POINT_ALL];


	float the1[DATA_POINT_ALL];
	float the2[DATA_POINT_ALL];
	float the3[DATA_POINT_ALL];


	int pwm1[DATA_POINT_ALL];
	int pwm2[DATA_POINT_ALL];
	int pwm3[DATA_POINT_ALL];


	float ang_inc = 0;
	for (int i=0; i<cw_array_length; i++){
	// for (int i=0; i<1; i++){

		XYZ_gen(1, ang_inc, x_arr, y_arr, z_arr);
		inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(crab_walking_LUT[i][0], pwm1, sizeof(crab_walking_LUT[i][0]));
		memcpy(crab_walking_LUT[i][1], pwm2, sizeof(crab_walking_LUT[i][1]));
		memcpy(crab_walking_LUT[i][2], pwm3, sizeof(crab_walking_LUT[i][2]));

		XYZ_gen(2, ang_inc, x_arr, y_arr, z_arr);
		inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(crab_walking_LUT[i][3], pwm1, sizeof(crab_walking_LUT[i][3]));
		memcpy(crab_walking_LUT[i][4], pwm2, sizeof(crab_walking_LUT[i][4]));
		memcpy(crab_walking_LUT[i][5], pwm3, sizeof(crab_walking_LUT[i][5]));

		XYZ_gen(3, ang_inc, x_arr, y_arr, z_arr);
		inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(crab_walking_LUT[i][6], pwm1, sizeof(crab_walking_LUT[i][6]));
		memcpy(crab_walking_LUT[i][7], pwm2, sizeof(crab_walking_LUT[i][7]));
		memcpy(crab_walking_LUT[i][8], pwm3, sizeof(crab_walking_LUT[i][8]));

		XYZ_gen(4, ang_inc, x_arr, y_arr, z_arr);
		inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(crab_walking_LUT[i][9], pwm1, sizeof(crab_walking_LUT[i][9]));
		memcpy(crab_walking_LUT[i][10], pwm2, sizeof(crab_walking_LUT[i][10]));
		memcpy(crab_walking_LUT[i][11], pwm3, sizeof(crab_walking_LUT[i][11]));

		XYZ_gen(5, ang_inc, x_arr, y_arr, z_arr);
		inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(crab_walking_LUT[i][12], pwm1, sizeof(crab_walking_LUT[i][12]));
		memcpy(crab_walking_LUT[i][13], pwm2, sizeof(crab_walking_LUT[i][13]));
		memcpy(crab_walking_LUT[i][14], pwm3, sizeof(crab_walking_LUT[i][14]));

		XYZ_gen(6, ang_inc, x_arr, y_arr, z_arr);
		inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(crab_walking_LUT[i][15], pwm1, sizeof(crab_walking_LUT[i][15]));
		memcpy(crab_walking_LUT[i][16], pwm2, sizeof(crab_walking_LUT[i][16]));
		memcpy(crab_walking_LUT[i][17], pwm3, sizeof(crab_walking_LUT[i][17]));

		ang_inc += cw_ang_resolution;
	}
}

void HexapodLib::matmul3_3(float A[3][3], float B[3][3], float C[3][3]){

	C[0][0] = A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0];
	C[0][1] = A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1];
	C[0][2] = A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2];

	C[1][0] = A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0];
	C[1][1] = A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1];
	C[1][2] = A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2];

	C[2][0] = A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0];
	C[2][1] = A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1];
	C[2][2] = A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2];
}

void HexapodLib::matmul3_1(float A[3][3], float B[3], float C[3]){

	C[0] = A[0][0]*B[0] + A[0][1]*B[1] + A[0][2]*B[2];
	C[1] = A[1][0]*B[0] + A[1][1]*B[1] + A[1][2]*B[2];
	C[2] = A[2][0]*B[0] + A[2][1]*B[1] + A[2][2]*B[2];

}

void HexapodLib::bodyRotate_to_newLegXYZ(float r, float p, float y, float* leg1_XYZ, float* leg2_XYZ, float* leg3_XYZ, float* leg4_XYZ, float* leg5_XYZ, float* leg6_XYZ){

	r = r*-1.0;
	p = p*-1.0;
	y = y*-1.0;

	/// Body Reverse Rotation ///
	float ROT_R11 = cos(y)*cos(p);
	float ROT_R12 = cos(y)*sin(p)*sin(r) - sin(y)*cos(r);
	float ROT_R13 = cos(y)*sin(p)*cos(r) + sin(y)*sin(r);

	float ROT_R21 = sin(y)*cos(p);
	float ROT_R22 = sin(y)*sin(p)*sin(r) + cos(y)*cos(r);
	float ROT_R23 = sin(y)*sin(p)*cos(r) - cos(y)*sin(r);

	float ROT_R31 = -sin(p);
	float ROT_R32 = cos(p)*sin(r);
	float ROT_R33 = cos(p)*cos(r);
	float REV_ROT_MUL[3][3] = {
		{ROT_R11, ROT_R12, ROT_R13}, 
		{ROT_R21, ROT_R22, ROT_R23},
		{ROT_R31, ROT_R32, ROT_R33}
	};

	/// Matrix multiplication of body rotation and leg offset rotation matrics
	float rot_O_to_leg1[3][3];
	float rot_O_to_leg2[3][3];
	float rot_O_to_leg3[3][3];
	float rot_O_to_leg4[3][3];
	float rot_O_to_leg5[3][3];
	float rot_O_to_leg6[3][3];

	matmul3_3(leg1_offset_rot, REV_ROT_MUL, rot_O_to_leg1);
	matmul3_3(leg2_offset_rot, REV_ROT_MUL, rot_O_to_leg2);
	matmul3_3(leg3_offset_rot, REV_ROT_MUL, rot_O_to_leg3);
	matmul3_3(leg4_offset_rot, REV_ROT_MUL, rot_O_to_leg4);
	matmul3_3(leg5_offset_rot, REV_ROT_MUL, rot_O_to_leg5);
	matmul3_3(leg6_offset_rot, REV_ROT_MUL, rot_O_to_leg6);

	// Homogenous transformation from XYZ home of O frame to leg frame
	matmul3_1(rot_O_to_leg1, XYZ_home_OI, leg1_XYZ);
	matmul3_1(rot_O_to_leg2, XYZ_home_OJ, leg2_XYZ);
	matmul3_1(rot_O_to_leg3, XYZ_home_OK, leg3_XYZ);
	matmul3_1(rot_O_to_leg4, XYZ_home_OL, leg4_XYZ);
	matmul3_1(rot_O_to_leg5, XYZ_home_OM, leg5_XYZ);
	matmul3_1(rot_O_to_leg6, XYZ_home_ON, leg6_XYZ);

	leg1_XYZ[0] = leg1_XYZ[0] + trans_O_to_leg1[0];
	leg1_XYZ[1] = leg1_XYZ[1] + trans_O_to_leg1[1];
	leg1_XYZ[2] = leg1_XYZ[2] + trans_O_to_leg1[2];

	leg2_XYZ[0] = leg2_XYZ[0] + trans_O_to_leg2[0];
	leg2_XYZ[1] = leg2_XYZ[1] + trans_O_to_leg2[1];
	leg2_XYZ[2] = leg2_XYZ[2] + trans_O_to_leg2[2];

	leg3_XYZ[0] = leg3_XYZ[0] + trans_O_to_leg3[0];
	leg3_XYZ[1] = leg3_XYZ[1] + trans_O_to_leg3[1];
	leg3_XYZ[2] = leg3_XYZ[2] + trans_O_to_leg3[2];

	leg4_XYZ[0] = leg4_XYZ[0] + trans_O_to_leg4[0];
	leg4_XYZ[1] = leg4_XYZ[1] + trans_O_to_leg4[1];
	leg4_XYZ[2] = leg4_XYZ[2] + trans_O_to_leg4[2];

	leg5_XYZ[0] = leg5_XYZ[0] + trans_O_to_leg5[0];
	leg5_XYZ[1] = leg5_XYZ[1] + trans_O_to_leg5[1];
	leg5_XYZ[2] = leg5_XYZ[2] + trans_O_to_leg5[2];

	leg6_XYZ[0] = leg6_XYZ[0] + trans_O_to_leg6[0];
	leg6_XYZ[1] = leg6_XYZ[1] + trans_O_to_leg6[1];
	leg6_XYZ[2] = leg6_XYZ[2] + trans_O_to_leg6[2];

}

void HexapodLib::bodyTranslate_to_newLegXYZ(float x, float y, float z, float* leg1_XYZ, float* leg2_XYZ, float* leg3_XYZ, float* leg4_XYZ, float* leg5_XYZ, float* leg6_XYZ){

	float PC_new[3] = {x, y, z};

	matmul3_1(leg1_offset_rot, PC_new, leg1_XYZ);
	matmul3_1(leg2_offset_rot, PC_new, leg2_XYZ);
	matmul3_1(leg3_offset_rot, PC_new, leg3_XYZ);
	matmul3_1(leg4_offset_rot, PC_new, leg4_XYZ);
	matmul3_1(leg5_offset_rot, PC_new, leg5_XYZ);
	matmul3_1(leg6_offset_rot, PC_new, leg6_XYZ);

	leg1_XYZ[0] = XYZ_home[0] - leg1_XYZ[0];
	leg1_XYZ[1] = XYZ_home[1] - leg1_XYZ[1];
	leg1_XYZ[2] = XYZ_home[2] - leg1_XYZ[2];

	leg2_XYZ[0] = XYZ_home[0] - leg2_XYZ[0];
	leg2_XYZ[1] = XYZ_home[1] - leg2_XYZ[1];
	leg2_XYZ[2] = XYZ_home[2] - leg2_XYZ[2];

	leg3_XYZ[0] = XYZ_home[0] - leg3_XYZ[0];
	leg3_XYZ[1] = XYZ_home[1] - leg3_XYZ[1];
	leg3_XYZ[2] = XYZ_home[2] - leg3_XYZ[2];

	leg4_XYZ[0] = XYZ_home[0] - leg4_XYZ[0];
	leg4_XYZ[1] = XYZ_home[1] - leg4_XYZ[1];
	leg4_XYZ[2] = XYZ_home[2] - leg4_XYZ[2];

	leg5_XYZ[0] = XYZ_home[0] - leg5_XYZ[0];
	leg5_XYZ[1] = XYZ_home[1] - leg5_XYZ[1];
	leg5_XYZ[2] = XYZ_home[2] - leg5_XYZ[2];

	leg6_XYZ[0] = XYZ_home[0] - leg6_XYZ[0];
	leg6_XYZ[1] = XYZ_home[1] - leg6_XYZ[1];
	leg6_XYZ[2] = XYZ_home[2] - leg6_XYZ[2];

}

void HexapodLib::generate_inplaceTurning(float turn_deg, float* x_IKM_combine, float* y_IKM_combine, float* z_IKM_combine, float* x_JLN_combine, float* y_JLN_combine, float* z_JLN_combine){

	float theta1_swing;
	float theta1_drag[DATA_POINT_TURN_PER_BLK];
	float theta2_drag[DATA_POINT_TURN_PER_BLK];
	float theta3_drag[DATA_POINT_TURN_PER_BLK];

	linspace(theta2_home, theta2_home, DATA_POINT_TURN_PER_BLK, theta2_drag);
	linspace(theta3_home, theta3_home, DATA_POINT_TURN_PER_BLK, theta3_drag);

	float x_LU_last = X_home;
	float y_LU_last = Y_home;
	float z_LU_last = -100.0;

	float x_LD_last = X_home;
	float y_LD_last = Y_home;
	float z_LD_last = Z_home;

	// xyz_LU_hold
	float x_LU_hold[DATA_POINT_TURN_PER_BLK];
	float y_LU_hold[DATA_POINT_TURN_PER_BLK];
	float z_LU_hold[DATA_POINT_TURN_PER_BLK];
	linspace(X_home, X_home, DATA_POINT_TURN_PER_BLK, x_LU_hold);
	linspace(Y_home, Y_home, DATA_POINT_TURN_PER_BLK, y_LU_hold);
	linspace(z_LU_last, z_LU_last, DATA_POINT_TURN_PER_BLK, z_LU_hold);

	// xyz_LU_move //
	float x_LD_move[DATA_POINT_TURN_PER_BLK];
	float y_LD_move[DATA_POINT_TURN_PER_BLK];
	float z_LD_move[DATA_POINT_TURN_PER_BLK];
	linspace(X_home, X_home, DATA_POINT_TURN_PER_BLK, x_LD_move);
	linspace(Y_home, Y_home, DATA_POINT_TURN_PER_BLK, y_LD_move);
	linspace(z_LU_last, Z_home, DATA_POINT_TURN_PER_BLK, z_LD_move);

	// xyz_LD_hold //
	float x_LD_hold[DATA_POINT_TURN_PER_BLK];
	float y_LD_hold[DATA_POINT_TURN_PER_BLK];
	float z_LD_hold[DATA_POINT_TURN_PER_BLK];
	linspace(X_home, X_home, DATA_POINT_TURN_PER_BLK, x_LD_hold);
	linspace(Y_home, Y_home, DATA_POINT_TURN_PER_BLK, y_LD_hold);
	linspace(Z_home, Z_home, DATA_POINT_TURN_PER_BLK, z_LD_hold);

	/// xyz_drag_move
	float x_drag_move[DATA_POINT_TURN_PER_BLK];
	float y_drag_move[DATA_POINT_TURN_PER_BLK];
	float z_drag_move[DATA_POINT_TURN_PER_BLK];

	/// xyz_drag_hold
	float x_drag_last;
	float y_drag_last;
	float z_drag_last;
	float x_drag_hold[DATA_POINT_TURN_PER_BLK];
	float y_drag_hold[DATA_POINT_TURN_PER_BLK];
	float z_drag_hold[DATA_POINT_TURN_PER_BLK];

	/// xyz_LU_move
	float x_LU_move[DATA_POINT_TURN_PER_BLK];
	float y_LU_move[DATA_POINT_TURN_PER_BLK];
	float z_LU_move[DATA_POINT_TURN_PER_BLK];


	theta1_swing = turn_deg*DEG2RAD;

	linspace(0, theta1_swing, DATA_POINT_TURN_PER_BLK, theta1_drag);

	/// xyz_drag_move
	for (int i=0; i<DATA_POINT_TURN_PER_BLK; i++){
		fwd(theta1_drag[i], theta2_drag[i], theta3_drag[i], x_drag_move[i], y_drag_move[i], z_drag_move[i]);
	}

	/// xyz_drag_hold
	x_drag_last = x_drag_move[DATA_POINT_TURN_PER_BLK-1];
	y_drag_last = y_drag_move[DATA_POINT_TURN_PER_BLK-1];
	z_drag_last = z_drag_move[DATA_POINT_TURN_PER_BLK-1];

	linspace(x_drag_last, x_drag_last, DATA_POINT_TURN_PER_BLK, x_drag_hold);
	linspace(y_drag_last, y_drag_last, DATA_POINT_TURN_PER_BLK, y_drag_hold);
	linspace(z_drag_last, z_drag_last, DATA_POINT_TURN_PER_BLK, z_drag_hold);

	/// xyz_LU_move
	linspace(x_drag_last, x_LU_last, DATA_POINT_TURN_PER_BLK, x_LU_move);
	linspace(y_drag_last, y_LU_last, DATA_POINT_TURN_PER_BLK, y_LU_move);
	linspace(z_drag_last, z_LU_last, DATA_POINT_TURN_PER_BLK, z_LU_move);

	/// BLK1 ///
	for (int j=0; j<DATA_POINT_TURN_PER_BLK; j++){
		x_IKM_combine[j] = x_LU_hold[j];
		y_IKM_combine[j] = y_LU_hold[j];
		z_IKM_combine[j] = z_LU_hold[j];

		x_JLN_combine[j] = x_drag_move[j];
		y_JLN_combine[j] = y_drag_move[j];
		z_JLN_combine[j] = z_drag_move[j];

	}
	/// BLK2 ///
	for (int j=0; j<DATA_POINT_TURN_PER_BLK; j++){
		x_IKM_combine[DATA_POINT_TURN_PER_BLK+j] = x_LD_move[j];
		y_IKM_combine[DATA_POINT_TURN_PER_BLK+j] = y_LD_move[j];
		z_IKM_combine[DATA_POINT_TURN_PER_BLK+j] = z_LD_move[j];

		x_JLN_combine[DATA_POINT_TURN_PER_BLK+j] = x_drag_hold[j];
		y_JLN_combine[DATA_POINT_TURN_PER_BLK+j] = y_drag_hold[j];
		z_JLN_combine[DATA_POINT_TURN_PER_BLK+j] = z_drag_hold[j];

	}
	/// BLK3 ///
	for (int j=0; j<DATA_POINT_TURN_PER_BLK; j++){
		x_IKM_combine[(int)(2*DATA_POINT_TURN_PER_BLK)+j] = x_LD_hold[j];
		y_IKM_combine[(int)(2*DATA_POINT_TURN_PER_BLK)+j] = y_LD_hold[j];
		z_IKM_combine[(int)(2*DATA_POINT_TURN_PER_BLK)+j] = z_LD_hold[j];

		x_JLN_combine[(int)(2*DATA_POINT_TURN_PER_BLK)+j] = x_LU_move[j];
		y_JLN_combine[(int)(2*DATA_POINT_TURN_PER_BLK)+j] = y_LU_move[j];
		z_JLN_combine[(int)(2*DATA_POINT_TURN_PER_BLK)+j] = z_LU_move[j];
	}
	/// BLK4 ///
	for (int j=0; j<DATA_POINT_TURN_PER_BLK; j++){
		x_IKM_combine[(int)(3*DATA_POINT_TURN_PER_BLK)+j] = x_drag_move[j];
		y_IKM_combine[(int)(3*DATA_POINT_TURN_PER_BLK)+j] = y_drag_move[j];
		z_IKM_combine[(int)(3*DATA_POINT_TURN_PER_BLK)+j] = z_drag_move[j];

		x_JLN_combine[(int)(3*DATA_POINT_TURN_PER_BLK)+j] = x_LU_hold[j];
		y_JLN_combine[(int)(3*DATA_POINT_TURN_PER_BLK)+j] = y_LU_hold[j];
		z_JLN_combine[(int)(3*DATA_POINT_TURN_PER_BLK)+j] = z_LU_hold[j];
	}
	/// BLK5 ///
	for (int j=0; j<DATA_POINT_TURN_PER_BLK; j++){
		x_IKM_combine[(int)(4*DATA_POINT_TURN_PER_BLK)+j] = x_drag_hold[j];
		y_IKM_combine[(int)(4*DATA_POINT_TURN_PER_BLK)+j] = y_drag_hold[j];
		z_IKM_combine[(int)(4*DATA_POINT_TURN_PER_BLK)+j] = z_drag_hold[j];

		x_JLN_combine[(int)(4*DATA_POINT_TURN_PER_BLK)+j] = x_LD_move[j];
		y_JLN_combine[(int)(4*DATA_POINT_TURN_PER_BLK)+j] = y_LD_move[j];
		z_JLN_combine[(int)(4*DATA_POINT_TURN_PER_BLK)+j] = z_LD_move[j];
	}
	/// BLK6 ///
	for (int j=0; j<DATA_POINT_TURN_PER_BLK; j++){
		x_IKM_combine[(int)(5*DATA_POINT_TURN_PER_BLK)+j] = x_LU_move[j];
		y_IKM_combine[(int)(5*DATA_POINT_TURN_PER_BLK)+j] = y_LU_move[j];
		z_IKM_combine[(int)(5*DATA_POINT_TURN_PER_BLK)+j] = z_LU_move[j];

		x_JLN_combine[(int)(5*DATA_POINT_TURN_PER_BLK)+j] = x_LD_hold[j];
		y_JLN_combine[(int)(5*DATA_POINT_TURN_PER_BLK)+j] = y_LD_hold[j];
		z_JLN_combine[(int)(5*DATA_POINT_TURN_PER_BLK)+j] = z_LD_hold[j];

	}
}

void HexapodLib::generate_inplaceTurning_LUT(){

	/// output ///
	float x_IKM_combine[DATA_POINT_TURN_ALL];
	float y_IKM_combine[DATA_POINT_TURN_ALL];
	float z_IKM_combine[DATA_POINT_TURN_ALL];

	float x_JLN_combine[DATA_POINT_TURN_ALL];
	float y_JLN_combine[DATA_POINT_TURN_ALL];
	float z_JLN_combine[DATA_POINT_TURN_ALL];

	float the1_1[DATA_POINT_TURN_ALL];
	float the2_1[DATA_POINT_TURN_ALL];
	float the3_1[DATA_POINT_TURN_ALL];

	float the1_2[DATA_POINT_TURN_ALL];
	float the2_2[DATA_POINT_TURN_ALL];
	float the3_2[DATA_POINT_TURN_ALL];

	int pwm1_1[DATA_POINT_TURN_ALL];
	int pwm2_1[DATA_POINT_TURN_ALL];
	int pwm3_1[DATA_POINT_TURN_ALL];

	int pwm1_2[DATA_POINT_TURN_ALL];
	int pwm2_2[DATA_POINT_TURN_ALL];
	int pwm3_2[DATA_POINT_TURN_ALL];


	for (int i=0; i<2; i++){

		if (i==0){
			generate_inplaceTurning(20.0, x_IKM_combine, y_IKM_combine, z_IKM_combine, x_JLN_combine, y_JLN_combine, z_JLN_combine);
		} else {
			generate_inplaceTurning(-20.0, x_IKM_combine, y_IKM_combine, z_IKM_combine, x_JLN_combine, y_JLN_combine, z_JLN_combine);
		}

		inv_arrays(x_IKM_combine, y_IKM_combine, z_IKM_combine, DATA_POINT_TURN_ALL, the1_1, the2_1, the3_1);
		inv_arrays(x_JLN_combine, y_JLN_combine, z_JLN_combine, DATA_POINT_TURN_ALL, the1_2, the2_2, the3_2);

		kinAngleArray_To_servoPwmArray(0, the1_1, DATA_POINT_TURN_ALL, pwm1_1);
		kinAngleArray_To_servoPwmArray(1, the2_1, DATA_POINT_TURN_ALL, pwm2_1);
		kinAngleArray_To_servoPwmArray(2, the3_1, DATA_POINT_TURN_ALL, pwm3_1);

		kinAngleArray_To_servoPwmArray(0, the1_2, DATA_POINT_TURN_ALL, pwm1_2);
		kinAngleArray_To_servoPwmArray(1, the2_2, DATA_POINT_TURN_ALL, pwm2_2);
		kinAngleArray_To_servoPwmArray(2, the3_2, DATA_POINT_TURN_ALL, pwm3_2);

		// leg I
		memcpy(inplace_turning_PWM_LUT[i][0], pwm1_1, sizeof(inplace_turning_PWM_LUT[i][0]));
		memcpy(inplace_turning_PWM_LUT[i][1], pwm2_1, sizeof(inplace_turning_PWM_LUT[i][1]));
		memcpy(inplace_turning_PWM_LUT[i][2], pwm3_1, sizeof(inplace_turning_PWM_LUT[i][2]));
		// leg J
		memcpy(inplace_turning_PWM_LUT[i][3], pwm1_2, sizeof(inplace_turning_PWM_LUT[i][3]));
		memcpy(inplace_turning_PWM_LUT[i][4], pwm2_2, sizeof(inplace_turning_PWM_LUT[i][4]));
		memcpy(inplace_turning_PWM_LUT[i][5], pwm3_2, sizeof(inplace_turning_PWM_LUT[i][5]));
		// leg K (same as leg I)
		memcpy(inplace_turning_PWM_LUT[i][6], pwm1_1, sizeof(inplace_turning_PWM_LUT[i][6]));
		memcpy(inplace_turning_PWM_LUT[i][7], pwm2_1, sizeof(inplace_turning_PWM_LUT[i][7]));
		memcpy(inplace_turning_PWM_LUT[i][8], pwm3_1, sizeof(inplace_turning_PWM_LUT[i][8]));
		// leg L (same as leg J)
		memcpy(inplace_turning_PWM_LUT[i][9],  pwm1_2, sizeof(inplace_turning_PWM_LUT[i][9]));
		memcpy(inplace_turning_PWM_LUT[i][10], pwm2_2, sizeof(inplace_turning_PWM_LUT[i][10]));
		memcpy(inplace_turning_PWM_LUT[i][11], pwm3_2, sizeof(inplace_turning_PWM_LUT[i][11]));
		// leg M (same as leg I)
		memcpy(inplace_turning_PWM_LUT[i][12], pwm1_1, sizeof(inplace_turning_PWM_LUT[i][12]));
		memcpy(inplace_turning_PWM_LUT[i][13], pwm2_1, sizeof(inplace_turning_PWM_LUT[i][13]));
		memcpy(inplace_turning_PWM_LUT[i][14], pwm3_1, sizeof(inplace_turning_PWM_LUT[i][14]));
		// leg N (same as leg J)
		memcpy(inplace_turning_PWM_LUT[i][15], pwm1_2, sizeof(inplace_turning_PWM_LUT[i][15]));
		memcpy(inplace_turning_PWM_LUT[i][16], pwm2_2, sizeof(inplace_turning_PWM_LUT[i][16]));
		memcpy(inplace_turning_PWM_LUT[i][17], pwm3_2, sizeof(inplace_turning_PWM_LUT[i][17]));

	}
}

void HexapodLib::generate_steering_curve(int leg_no, int str_sign, float R_icc, float leg_XYZ[DATA_POINT_ALL][3]){

	float ICC_offset[3] = {-str_sign*R_icc, 0.0, 0.0};
	float str_ang = (curve_path)/(R_icc + C1 + X_home);
	float leg_STR_Z_line[DATA_POINT_PER_LINE];
	float t[DATA_POINT_PER_LINE];

	linspace(S, S, DATA_POINT_PER_LINE, leg_STR_Z_line);
	linspace(0.0, 1.0, DATA_POINT_PER_LINE, t);
	float leg_P1[2];
	float leg_P2[2];
	float leg_P3[2];
	float leg_offset_ang;
	float Leg_ang_array[DATA_POINT_PER_LINE];
	float leg_X_line[DATA_POINT_PER_LINE];
	float leg_Y_line[DATA_POINT_PER_LINE];
	float leg_X_curve[DATA_POINT_PER_LINE];
	float leg_Y_curve[DATA_POINT_PER_LINE];
	float leg_Z_curve[DATA_POINT_PER_LINE];
	float leg_X[DATA_POINT_ALL];
	float leg_Y[DATA_POINT_ALL];
	float leg_Z[DATA_POINT_ALL];

	float matmul_result[3] = {0,0,0};
	float xyz_O[3] = {0,0,0};

	if (leg_no == 1){

		/////////////
		/// Leg I ///
		/////////////
		linspace(str_ang, -str_ang, DATA_POINT_PER_LINE, Leg_ang_array);

		for (int i=0; i<DATA_POINT_PER_LINE; i++){
			leg_X_line[i] = str_sign*(R_icc + (str_sign*(C1 + X_home)))*cos(Leg_ang_array[i]);
			leg_Y_line[i] = (R_icc + (str_sign*(C1 + X_home)))*sin(Leg_ang_array[i]);
			leg_X_curve[i] = leg_X_line[i];
		}

		// float legI_x_start = legI_X_line[0];
		// float legI_x_end = legI_X_line[DATA_POINT_PER_LINE-1];
		// float legI_y_start = legI_Y_line[0];
		// float legI_y_end = legI_Y_line[DATA_POINT_PER_LINE-1];

		leg_P1[0] = leg_Y_line[DATA_POINT_PER_LINE-1];
		leg_P1[1] = S;
		leg_P2[0] = 0;
		leg_P2[1] = (S+(2*A));
		leg_P3[0] = leg_Y_line[0];
		leg_P3[1] = S;

		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			leg_Y_curve[i] = (pow((1 - t[i]), 2) * leg_P1[0]) + (2 * (1 - t[i]) * t[i] * leg_P2[0]) + (pow(t[i], 2) * leg_P3[0]);
			leg_Z_curve[i] = (pow((1 - t[i]), 2) * leg_P1[1]) + (2 * (1 - t[i]) * t[i] * leg_P2[1]) + (pow(t[i], 2) * leg_P3[1]);
		}

		if (str_sign == 1){
			concat_arrays(leg_X_curve, leg_X_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_curve, leg_Y_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_Z_curve, leg_STR_Z_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		} else {
			concat_arrays(leg_X_line, leg_X_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_line, leg_Y_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_STR_Z_line, leg_Z_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		}
		// applying offset
		// for (int i = 0; i < DATA_POINT_ALL; i++){
		// 	leg_X[i] = leg_X[i] + ICC_offset[0];
		// }

		for (int i=0; i<DATA_POINT_ALL; i++){
			// float matmul_result[3];
			xyz_O[0] = leg_X[i] + ICC_offset[0];
			xyz_O[1] = leg_Y[i];
			xyz_O[2] = leg_Z[i];
			matmul3_1(leg1_offset_rot, xyz_O, matmul_result);
			leg_XYZ[i][0] = matmul_result[0] - C1;
			leg_XYZ[i][1] = matmul_result[1];
			leg_XYZ[i][2] = matmul_result[2];

		}

  } else if (leg_no == 4){

		/////////////
		/// Leg L ///
		/////////////
		leg_offset_ang = 0;
		linspace(str_ang, -str_ang, DATA_POINT_PER_LINE, Leg_ang_array);


		for (int i=0; i<DATA_POINT_PER_LINE; i++){
			leg_X_line[i] = str_sign*(R_icc - (str_sign*(C1 + X_home)))*cos(Leg_ang_array[i]);
			leg_Y_line[i] = (R_icc - (str_sign*(C1 + X_home)))*sin(Leg_ang_array[i]);
			leg_X_curve[i] = leg_X_line[i];
		}

		leg_P1[0] = leg_Y_line[DATA_POINT_PER_LINE-1];
		leg_P1[1] = S;
		leg_P2[0] = 0;
		leg_P2[1] = (S+(2*A));
		leg_P3[0] = leg_Y_line[0];
		leg_P3[1] = S;

		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			leg_Y_curve[i] = (pow((1 - t[i]), 2) * leg_P1[0]) + (2 * (1 - t[i]) * t[i] * leg_P2[0]) + (pow(t[i], 2) * leg_P3[0]);
			leg_Z_curve[i] = (pow((1 - t[i]), 2) * leg_P1[1]) + (2 * (1 - t[i]) * t[i] * leg_P2[1]) + (pow(t[i], 2) * leg_P3[1]);
		}

		if (str_sign == 1){
			concat_arrays(leg_X_line, leg_X_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_line, leg_Y_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_STR_Z_line, leg_Z_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		} else {
			concat_arrays(leg_X_curve, leg_X_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_curve, leg_Y_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_Z_curve, leg_STR_Z_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		}
		// applying offset
		// for (int i = 0; i < DATA_POINT_ALL; i++){
		// 	leg_X[i] = leg_X[i] + ICC_offset[0];
		// }

		for (int i=0; i<DATA_POINT_ALL; i++){
			// float matmul_result[3];
			// float xyz_O[3] = {legL_X[i], legL_Y[i], legL_Z[i]};
			xyz_O[0] = leg_X[i] + ICC_offset[0];
			xyz_O[1] = leg_Y[i];
			xyz_O[2] = leg_Z[i];
			matmul3_1(leg4_offset_rot, xyz_O, matmul_result);
			leg_XYZ[i][0] = matmul_result[0] - C1;
			leg_XYZ[i][1] = matmul_result[1];
			leg_XYZ[i][2] = matmul_result[2];
			
		}


  } else if (leg_no == 2){

		/////////////
		/// Leg J ///
		/////////////
		leg_offset_ang = atan(((C2+X_home)*sin(beta_rad)) / (R_icc + (str_sign*(C2+X_home)*cos(beta_rad))));
		linspace(leg_offset_ang+str_ang, leg_offset_ang-str_ang, DATA_POINT_PER_LINE, Leg_ang_array);

		for (int i=0; i<DATA_POINT_PER_LINE; i++){
			leg_X_line[i] = str_sign*(R_icc + (str_sign*(C2 + X_home)*cos(beta_rad)))*cos(Leg_ang_array[i]);
			leg_Y_line[i] = (R_icc + (str_sign*(C2 + X_home)*cos(beta_rad)))*sin(Leg_ang_array[i]);
			leg_X_curve[DATA_POINT_PER_LINE-1-i] = leg_X_line[i];
		}

		// float legJ_x_start = legJ_X_line[0];
		// float legJ_x_end = legJ_X_line[DATA_POINT_PER_LINE-1];
		// float legJ_y_start = legJ_Y_line[DATA_POINT_PER_LINE-1]; 
		// float legJ_y_end = legJ_Y_line[0];
		// float legJ_y_mid = (legJ_Y_line[DATA_POINT_PER_LINE-1] + legJ_Y_line[0])/2;

		leg_P1[0] = leg_Y_line[0];
		leg_P1[1] = S;
		leg_P2[0] = (leg_Y_line[DATA_POINT_PER_LINE-1] + leg_Y_line[0])/2;
		leg_P2[1] = (S+(2*A));
		leg_P3[0] = leg_Y_line[DATA_POINT_PER_LINE-1];
		leg_P3[1] = S;


		// float legJ_P1[2] = {legJ_y_end, S};
		// float legJ_P2[2] = {legJ_y_mid, (S+(2*A))};
		// float legJ_P3[2] = {legJ_y_start, S};

		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			leg_Y_curve[DATA_POINT_PER_LINE-1-i] = (pow((1 - t[i]), 2) * leg_P1[0]) + (2 * (1 - t[i]) * t[i] * leg_P2[0]) + (pow(t[i], 2) * leg_P3[0]);
			leg_Z_curve[i] = (pow((1 - t[i]), 2) * leg_P1[1]) + (2 * (1 - t[i]) * t[i] * leg_P2[1]) + (pow(t[i], 2) * leg_P3[1]);
		}

		if (str_sign == 1){
			concat_arrays(leg_X_line, leg_X_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_line, leg_Y_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_STR_Z_line, leg_Z_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		} else {
			concat_arrays(leg_X_curve, leg_X_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_curve, leg_Y_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_Z_curve, leg_STR_Z_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		}
		// applying offset
		// for (int i = 0; i < DATA_POINT_ALL; i++){
		// 	leg_X[i] = leg_X[i] + ICC_offset[0];
		// }

		for (int i=0; i<DATA_POINT_ALL; i++){
			// float matmul_result[3];
			// float xyz_O[3] = {legJ_X[i], legJ_Y[i], legJ_Z[i]};
			xyz_O[0] = leg_X[i] + ICC_offset[0];
			xyz_O[1] = leg_Y[i];
			xyz_O[2] = leg_Z[i];
			matmul3_1(leg2_offset_rot, xyz_O, matmul_result);
			leg_XYZ[i][0] = matmul_result[0] - C2;
			leg_XYZ[i][1] = matmul_result[1];
			leg_XYZ[i][2] = matmul_result[2];

		}


  } else if (leg_no == 3){

		/////////////
		/// Leg K ///
		/////////////
		leg_offset_ang = atan(((C2+X_home)*sin(beta_rad)) / (R_icc - (str_sign*(C2+X_home)*cos(beta_rad))));
		linspace(leg_offset_ang+str_ang, leg_offset_ang-str_ang, DATA_POINT_PER_LINE, Leg_ang_array);

		for (int i=0; i<DATA_POINT_PER_LINE; i++){
			leg_X_line[i] = str_sign*(R_icc - (str_sign*(C2 + X_home)*cos(beta_rad)))*cos(Leg_ang_array[i]);
			leg_Y_line[i] = (R_icc - (str_sign*(C2 + X_home)*cos(beta_rad)))*sin(Leg_ang_array[i]);
			leg_X_curve[DATA_POINT_PER_LINE-1-i] = leg_X_line[i];
		}

		// float legK_x_start = legK_X_line[0];
		// float legK_x_end = legK_X_line[DATA_POINT_PER_LINE-1];
		// float legK_y_start = legK_Y_line[DATA_POINT_PER_LINE-1]; 
		// float legK_y_end = legK_Y_line[0];
		// float legK_y_mid = (legK_Y_line[DATA_POINT_PER_LINE-1] + legK_Y_line[0])/2;

		leg_P1[0] = leg_Y_line[0];
		leg_P1[1] = S;
		leg_P2[0] = (leg_Y_line[DATA_POINT_PER_LINE-1] + leg_Y_line[0])/2;
		leg_P2[1] = (S+(2*A));
		leg_P3[0] = leg_Y_line[DATA_POINT_PER_LINE-1];
		leg_P3[1] = S;

		// float legK_P1[2] = {legK_y_end, S};
		// float legK_P2[2] = {legK_y_mid, (S+(2*A))};
		// float legK_P3[2] = {legK_y_start, S};

		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			leg_Y_curve[DATA_POINT_PER_LINE-1-i] = (pow((1 - t[i]), 2) * leg_P1[0]) + (2 * (1 - t[i]) * t[i] * leg_P2[0]) + (pow(t[i], 2) * leg_P3[0]);
			leg_Z_curve[i] = (pow((1 - t[i]), 2) * leg_P1[1]) + (2 * (1 - t[i]) * t[i] * leg_P2[1]) + (pow(t[i], 2) * leg_P3[1]);
		}

		if (str_sign == 1){
			concat_arrays(leg_X_curve, leg_X_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_curve, leg_Y_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_Z_curve, leg_STR_Z_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		} else {
			concat_arrays(leg_X_line, leg_X_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_line, leg_Y_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_STR_Z_line, leg_Z_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		}
		// applying offset
		// for (int i = 0; i < DATA_POINT_ALL; i++){
		// 	leg_X[i] = leg_X[i] + ICC_offset[0];
		// }

		//float legK_K_XYZ[DATA_POINT_ALL][3];

		for (int i=0; i<DATA_POINT_ALL; i++){
			// float matmul_result[3];
			// float xyz_O[3] = {legK_X[i], legK_Y[i], legK_Z[i]};
			xyz_O[0] = leg_X[i] + ICC_offset[0];
			xyz_O[1] = leg_Y[i];
			xyz_O[2] = leg_Z[i];
			matmul3_1(leg3_offset_rot, xyz_O, matmul_result);
			leg_XYZ[i][0] = matmul_result[0] - C2;
			leg_XYZ[i][1] = matmul_result[1];
			leg_XYZ[i][2] = matmul_result[2];
		}


  } else if (leg_no == 5){

		/////////////
		/// Leg M ///
		/////////////
		leg_offset_ang = atan((-(C2+X_home)*sin(beta_rad)) / (R_icc - (str_sign*(C2+X_home)*cos(beta_rad))));
		linspace(leg_offset_ang+str_ang, leg_offset_ang-str_ang, DATA_POINT_PER_LINE, Leg_ang_array);

		for (int i=0; i<DATA_POINT_PER_LINE; i++){
			leg_X_line[i] = str_sign*(R_icc - (str_sign*(C2 + X_home)*cos(beta_rad)))*cos(Leg_ang_array[i]);
			leg_Y_line[i] = (R_icc - (str_sign*(C2 + X_home)*cos(beta_rad)))*sin(Leg_ang_array[i]);
			leg_X_curve[DATA_POINT_PER_LINE-1-i] = leg_X_line[i];
		}

		// float legM_x_start = legM_X_line[0];
		// float legM_x_end = legM_X_line[DATA_POINT_PER_LINE-1];
		// float legM_y_start = legM_Y_line[DATA_POINT_PER_LINE-1]; 
		// float legM_y_end = legM_Y_line[0];
		// float legM_y_mid = (legM_Y_line[DATA_POINT_PER_LINE-1] + legM_Y_line[0])/2;

		leg_P1[0] = leg_Y_line[0];
		leg_P1[1] = S;
		leg_P2[0] = (leg_Y_line[DATA_POINT_PER_LINE-1] + leg_Y_line[0])/2;
		leg_P2[1] = (S+(2*A));
		leg_P3[0] = leg_Y_line[DATA_POINT_PER_LINE-1];
		leg_P3[1] = S;

		// float legM_P1[2] = {legM_y_end, S};
		// float legM_P2[2] = {legM_y_mid, (S+(2*A))};
		// float legM_P3[2] = {legM_y_start, S};

		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			leg_Y_curve[DATA_POINT_PER_LINE-1-i] = (pow((1 - t[i]), 2) * leg_P1[0]) + (2 * (1 - t[i]) * t[i] * leg_P2[0]) + (pow(t[i], 2) * leg_P3[0]);
			leg_Z_curve[i] = (pow((1 - t[i]), 2) * leg_P1[1]) + (2 * (1 - t[i]) * t[i] * leg_P2[1]) + (pow(t[i], 2) * leg_P3[1]);
		}

		if (str_sign == 1){
			concat_arrays(leg_X_curve, leg_X_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_curve, leg_Y_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_Z_curve, leg_STR_Z_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		} else {
			concat_arrays(leg_X_line, leg_X_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_line, leg_Y_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_STR_Z_line, leg_Z_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		}
		// applying offset
		// for (int i = 0; i < DATA_POINT_ALL; i++){
		// 	leg_X[i] = leg_X[i] + ICC_offset[0];
		// }

		for (int i=0; i<DATA_POINT_ALL; i++){
			// float matmul_result[3];
			// float xyz_O[3] = {legM_X[i], legM_Y[i], legM_Z[i]};
			xyz_O[0] = leg_X[i] + ICC_offset[0];
			xyz_O[1] = leg_Y[i];
			xyz_O[2] = leg_Z[i];
			matmul3_1(leg5_offset_rot, xyz_O, matmul_result);
			leg_XYZ[i][0] = matmul_result[0] - C2;
			leg_XYZ[i][1] = matmul_result[1];
			leg_XYZ[i][2] = matmul_result[2];
			
		}


  } else if (leg_no == 6){

		/////////////
		/// Leg N ///
		/////////////
		leg_offset_ang = atan((-(C2+X_home)*sin(beta_rad)) / (R_icc + (str_sign*(C2+X_home)*cos(beta_rad))));
		linspace(leg_offset_ang+str_ang, leg_offset_ang-str_ang, DATA_POINT_PER_LINE, Leg_ang_array);

		for (int i=0; i<DATA_POINT_PER_LINE; i++){
			leg_X_line[i] = str_sign*(R_icc + (str_sign*(C2 + X_home)*cos(beta_rad)))*cos(Leg_ang_array[i]);
			leg_Y_line[i] = (R_icc + (str_sign*(C2 + X_home)*cos(beta_rad)))*sin(Leg_ang_array[i]);
			leg_X_curve[DATA_POINT_PER_LINE-1-i] = leg_X_line[i];
		}

		// float legN_x_start = legN_X_line[0];
		// float legN_x_end = legN_X_line[DATA_POINT_PER_LINE-1];
		// float legN_y_start = legN_Y_line[DATA_POINT_PER_LINE-1]; 
		// float legN_y_end = legN_Y_line[0];
		// float legN_y_mid = (legN_Y_line[DATA_POINT_PER_LINE-1] + legN_Y_line[0])/2;

		leg_P1[0] = leg_Y_line[0];
		leg_P1[1] = S;
		leg_P2[0] = (leg_Y_line[DATA_POINT_PER_LINE-1] + leg_Y_line[0])/2;
		leg_P2[1] = (S+(2*A));
		leg_P3[0] = leg_Y_line[DATA_POINT_PER_LINE-1];
		leg_P3[1] = S;

		// float legN_P1[2] = {legN_y_end, S};
		// float legN_P2[2] = {legN_y_mid, (S+(2*A))};
		// float legN_P3[2] = {legN_y_start, S};

		for (int i = 0; i < DATA_POINT_PER_LINE; i++) {
			leg_Y_curve[DATA_POINT_PER_LINE-1-i] = (pow((1 - t[i]), 2) * leg_P1[0]) + (2 * (1 - t[i]) * t[i] * leg_P2[0]) + (pow(t[i], 2) * leg_P3[0]);
			leg_Z_curve[i] = (pow((1 - t[i]), 2) * leg_P1[1]) + (2 * (1 - t[i]) * t[i] * leg_P2[1]) + (pow(t[i], 2) * leg_P3[1]);
		}

		if (str_sign == 1){
			concat_arrays(leg_X_line, leg_X_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_line, leg_Y_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_STR_Z_line, leg_Z_curve, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		} else {
			concat_arrays(leg_X_curve, leg_X_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_X);
			concat_arrays(leg_Y_curve, leg_Y_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Y);
			concat_arrays(leg_Z_curve, leg_STR_Z_line, DATA_POINT_PER_LINE, DATA_POINT_PER_LINE, leg_Z);
		}
		// applying offset
		// for (int i = 0; i < DATA_POINT_ALL; i++){
		// 	leg_X[i] = leg_X[i] + ICC_offset[0];
		// }

		for (int i=0; i<DATA_POINT_ALL; i++){
			// float matmul_result[3];
			// float xyz_O[3] = {legN_X[i], legN_Y[i], legN_Z[i]};
			xyz_O[0] = leg_X[i] + ICC_offset[0];
			xyz_O[1] = leg_Y[i];
			xyz_O[2] = leg_Z[i];
			matmul3_1(leg6_offset_rot, xyz_O, matmul_result);
			leg_XYZ[i][0] = matmul_result[0] - C2;
			leg_XYZ[i][1] = matmul_result[1];
			leg_XYZ[i][2] = matmul_result[2];
		}
	}


}

void HexapodLib::generate_normalWalking_LUT(){

	float left_R_icc_array[7];
	float right_R_icc_array[7];

	linspace(R_icc_min, R_icc_max, 7, left_R_icc_array);
	linspace(R_icc_max, R_icc_min, 7, right_R_icc_array);

	float leg_XYZ[DATA_POINT_ALL][3];

	float x_arr[DATA_POINT_ALL];
	float y_arr[DATA_POINT_ALL];
	float z_arr[DATA_POINT_ALL];

	float the1[DATA_POINT_ALL];
	float the2[DATA_POINT_ALL];
	float the3[DATA_POINT_ALL];

	int pwm1[DATA_POINT_ALL];
	int pwm2[DATA_POINT_ALL];
	int pwm3[DATA_POINT_ALL];

	/// Steer to lef CCW //
	for (int i=0; i < 7; i++){
		generate_steering_curve(1, 1, left_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[i][0],  pwm1, sizeof(normalWalking_PWM_LUT[i][0]));
		memcpy(normalWalking_PWM_LUT[i][1],  pwm2, sizeof(normalWalking_PWM_LUT[i][1]));
		memcpy(normalWalking_PWM_LUT[i][2],  pwm3, sizeof(normalWalking_PWM_LUT[i][2]));

		generate_steering_curve(2, 1, left_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[i][3],  pwm1, sizeof(normalWalking_PWM_LUT[i][3]));
		memcpy(normalWalking_PWM_LUT[i][4],  pwm2, sizeof(normalWalking_PWM_LUT[i][4]));
		memcpy(normalWalking_PWM_LUT[i][5],  pwm3, sizeof(normalWalking_PWM_LUT[i][5]));

		generate_steering_curve(3, 1, left_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[i][6],  pwm1, sizeof(normalWalking_PWM_LUT[i][6]));
		memcpy(normalWalking_PWM_LUT[i][7],  pwm2, sizeof(normalWalking_PWM_LUT[i][7]));
		memcpy(normalWalking_PWM_LUT[i][8],  pwm3, sizeof(normalWalking_PWM_LUT[i][8]));

		generate_steering_curve(4, 1, left_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[i][9],  pwm1, sizeof(normalWalking_PWM_LUT[i][9]));
		memcpy(normalWalking_PWM_LUT[i][10], pwm2, sizeof(normalWalking_PWM_LUT[i][10]));
		memcpy(normalWalking_PWM_LUT[i][11], pwm3, sizeof(normalWalking_PWM_LUT[i][11]));

		generate_steering_curve(5, 1, left_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[i][12], pwm1, sizeof(normalWalking_PWM_LUT[i][12]));
		memcpy(normalWalking_PWM_LUT[i][13], pwm2, sizeof(normalWalking_PWM_LUT[i][13]));
		memcpy(normalWalking_PWM_LUT[i][14], pwm3, sizeof(normalWalking_PWM_LUT[i][14]));

		generate_steering_curve(6, 1, left_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[i][15], pwm1, sizeof(normalWalking_PWM_LUT[i][15]));
		memcpy(normalWalking_PWM_LUT[i][16], pwm2, sizeof(normalWalking_PWM_LUT[i][16]));
		memcpy(normalWalking_PWM_LUT[i][17], pwm3, sizeof(normalWalking_PWM_LUT[i][17]));

	}

	/// Straight ///
	XYZ_gen(1, 0, x_arr, y_arr, z_arr);
	inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
	kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
	kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
	kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
	memcpy(normalWalking_PWM_LUT[7][0],  pwm1, sizeof(normalWalking_PWM_LUT[7][0]));
	memcpy(normalWalking_PWM_LUT[7][1],  pwm2, sizeof(normalWalking_PWM_LUT[7][1]));
	memcpy(normalWalking_PWM_LUT[7][2],  pwm3, sizeof(normalWalking_PWM_LUT[7][2]));

	XYZ_gen(2, 0, x_arr, y_arr, z_arr);
	inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
	kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
	kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
	kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
	memcpy(normalWalking_PWM_LUT[7][3],  pwm1, sizeof(normalWalking_PWM_LUT[7][3]));
	memcpy(normalWalking_PWM_LUT[7][4],  pwm2, sizeof(normalWalking_PWM_LUT[7][4]));
	memcpy(normalWalking_PWM_LUT[7][5],  pwm3, sizeof(normalWalking_PWM_LUT[7][5]));

	XYZ_gen(3, 0, x_arr, y_arr, z_arr);
	inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
	kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
	kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
	kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
	memcpy(normalWalking_PWM_LUT[7][6],  pwm1, sizeof(normalWalking_PWM_LUT[7][6]));
	memcpy(normalWalking_PWM_LUT[7][7],  pwm2, sizeof(normalWalking_PWM_LUT[7][7]));
	memcpy(normalWalking_PWM_LUT[7][8],  pwm3, sizeof(normalWalking_PWM_LUT[7][8]));

	XYZ_gen(4, 0, x_arr, y_arr, z_arr);
	inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
	kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
	kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
	kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
	memcpy(normalWalking_PWM_LUT[7][9],  pwm1, sizeof(normalWalking_PWM_LUT[7][9]));
	memcpy(normalWalking_PWM_LUT[7][10], pwm2, sizeof(normalWalking_PWM_LUT[7][10]));
	memcpy(normalWalking_PWM_LUT[7][11], pwm3, sizeof(normalWalking_PWM_LUT[7][11]));

	XYZ_gen(5, 0, x_arr, y_arr, z_arr);
	inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
	kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
	kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
	kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
	memcpy(normalWalking_PWM_LUT[7][12], pwm1, sizeof(normalWalking_PWM_LUT[7][12]));
	memcpy(normalWalking_PWM_LUT[7][13], pwm2, sizeof(normalWalking_PWM_LUT[7][13]));
	memcpy(normalWalking_PWM_LUT[7][14], pwm3, sizeof(normalWalking_PWM_LUT[7][14]));

	XYZ_gen(6, 0, x_arr, y_arr, z_arr);
	inv_arrays(x_arr, y_arr, z_arr, DATA_POINT_ALL, the1, the2, the3);
	kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
	kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
	kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
	memcpy(normalWalking_PWM_LUT[7][15], pwm1, sizeof(normalWalking_PWM_LUT[7][15]));
	memcpy(normalWalking_PWM_LUT[7][16], pwm2, sizeof(normalWalking_PWM_LUT[7][16]));
	memcpy(normalWalking_PWM_LUT[7][17], pwm3, sizeof(normalWalking_PWM_LUT[7][17]));



	/// Steer to right CW //
	for (int i=0; i < 7; i++){
		generate_steering_curve(1, -1, right_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[8+i][0],  pwm1, sizeof(normalWalking_PWM_LUT[8+i][0]));
		memcpy(normalWalking_PWM_LUT[8+i][1],  pwm2, sizeof(normalWalking_PWM_LUT[8+i][1]));
		memcpy(normalWalking_PWM_LUT[8+i][2],  pwm3, sizeof(normalWalking_PWM_LUT[8+i][2]));

		generate_steering_curve(2, -1, right_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[8+i][3],  pwm1, sizeof(normalWalking_PWM_LUT[8+i][3]));
		memcpy(normalWalking_PWM_LUT[8+i][4],  pwm2, sizeof(normalWalking_PWM_LUT[8+i][4]));
		memcpy(normalWalking_PWM_LUT[8+i][5],  pwm3, sizeof(normalWalking_PWM_LUT[8+i][5]));

		generate_steering_curve(3, -1, right_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[8+i][6],  pwm1, sizeof(normalWalking_PWM_LUT[8+i][6]));
		memcpy(normalWalking_PWM_LUT[8+i][7],  pwm2, sizeof(normalWalking_PWM_LUT[8+i][7]));
		memcpy(normalWalking_PWM_LUT[8+i][8],  pwm3, sizeof(normalWalking_PWM_LUT[8+i][8]));

		generate_steering_curve(4, -1, right_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[8+i][9],  pwm1, sizeof(normalWalking_PWM_LUT[8+i][9]));
		memcpy(normalWalking_PWM_LUT[8+i][10], pwm2, sizeof(normalWalking_PWM_LUT[8+i][10]));
		memcpy(normalWalking_PWM_LUT[8+i][11], pwm3, sizeof(normalWalking_PWM_LUT[8+i][11]));

		generate_steering_curve(5, -1, right_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[8+i][12], pwm1, sizeof(normalWalking_PWM_LUT[8+i][12]));
		memcpy(normalWalking_PWM_LUT[8+i][13], pwm2, sizeof(normalWalking_PWM_LUT[8+i][13]));
		memcpy(normalWalking_PWM_LUT[8+i][14], pwm3, sizeof(normalWalking_PWM_LUT[8+i][14]));

		generate_steering_curve(6, -1, right_R_icc_array[i], leg_XYZ);
		invKinArray_to_ThetaDegArray(leg_XYZ, the1, the2, the3);
		kinAngleArray_To_servoPwmArray(0, the1, DATA_POINT_ALL, pwm1);
		kinAngleArray_To_servoPwmArray(1, the2, DATA_POINT_ALL, pwm2);
		kinAngleArray_To_servoPwmArray(2, the3, DATA_POINT_ALL, pwm3);
		memcpy(normalWalking_PWM_LUT[8+i][15], pwm1, sizeof(normalWalking_PWM_LUT[8+i][15]));
		memcpy(normalWalking_PWM_LUT[8+i][16], pwm2, sizeof(normalWalking_PWM_LUT[8+i][16]));
		memcpy(normalWalking_PWM_LUT[8+i][17], pwm3, sizeof(normalWalking_PWM_LUT[8+i][17]));

	}
}