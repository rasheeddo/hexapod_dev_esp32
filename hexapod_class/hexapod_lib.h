#ifndef HexapodLib_H
#define HexapodLib_H

#include "Arduino.h"

#define RAD2DEG 180/PI
#define DEG2RAD PI/180
#define DATA_POINT_PER_LINE 20
#define DATA_POINT_ALL DATA_POINT_PER_LINE*2
#define DATA_POINT_TURN_PER_BLK 5
#define DATA_POINT_TURN_ALL DATA_POINT_TURN_PER_BLK*6

class HexapodLib {

private:

	/////////////
	/// Servo ///
	/////////////
	/*	
		pwmX_min : minimum PWM value for that X servo
		pwmX_mid : mid PWM value for that X servo
		pwmX_max : maximum PWM value for that X servo
		jointX_min : minimum kinematics angle in degree of joint X
		jointX_max : maximum kinematics angle in degree of joint X
	*/

	///////////////////
	/// Legs length ///
	///////////////////
	float L1 = 60;
	float L2 = 80;
	float L3 = 171;

	//////////////////////////
	/// walking parameters ///
	//////////////////////////
	/*
		S : starting point offset from 0
		T : step distance (how far the foot will move)
		A : step height (how the foot will lift from ground)
		beta : angle between ci and cj (1st leg and 2nd leg)
		x_start : X starting position before walking
	*/
	/// foot's path //
	float S = -150; // starting point offset from 0 (in Z-axis)
	float T = 100;  // step distance (how far the foot will move in Y-axis)
	float A = 50;   // step height (how the foot will lift from ground in Z-axis)

	float x_start = 160.0;

	/// walk rotation matrix ///
	// rotation angle to rotation foot path of each leg
	// the line+curve points have to rotate according to each leg
	float beta = 55.71;
	float beta_rad = beta * DEG2RAD;
	float walk_rot_ang1 = 0.0;
	float walk_rot_ang2 = -beta * DEG2RAD;
	float walk_rot_ang3 = beta * DEG2RAD;
	float walk_rot_ang4 = 180.0 * DEG2RAD;
	float walk_rot_ang5 = -beta * DEG2RAD;
	float walk_rot_ang6 = beta * DEG2RAD;

	///////////////////////////////////
	/// Body translation & rotation ///
	///////////////////////////////////
	// C1 : distance between body center point C to leg i,l frame
	// C2 : distance between body center point C to leg j,k,m,n frame
	float C1 = 118.4;
	float C2 = 143.54;

	// X_home, Y_home, Z_home : home position of foot tip in leg frame
	// theta1,2,3_home : home angle of leg (kinematics angle) 
	float X_home = 160.0;
	float Y_home = 0.0;
	float Z_home = -150.0;
	float theta1_home = 0.0;
	float theta2_home = 14.13 * DEG2RAD;
	float theta3_home = -96.59 * DEG2RAD;
	float XYZ_home[3] = {X_home, Y_home, Z_home};

	/// Leg's angle offset and rotation matrices /// 
	float leg1_ang_offset = 0.0;
	float leg2_ang_offset = -beta * DEG2RAD;
	float leg3_ang_offset = -(180.0 - beta) * DEG2RAD;
	float leg4_ang_offset = 180.0 * DEG2RAD;
	float leg5_ang_offset = (180.0 - beta) * DEG2RAD;
	float leg6_ang_offset = beta * DEG2RAD;

	float leg1_offset_rot[3][3] = {
		{cos(leg1_ang_offset), -sin(leg1_ang_offset), 0.0}, 
		{sin(leg1_ang_offset), cos(leg1_ang_offset), 0.0},
		{0.0, 0.0, 1.0}
	};

	float leg2_offset_rot[3][3] = {
		{cos(leg2_ang_offset), -sin(leg2_ang_offset), 0.0}, 
		{sin(leg2_ang_offset), cos(leg2_ang_offset), 0.0},
		{0.0, 0.0, 1.0}
	};

	float leg3_offset_rot[3][3] = {
		{cos(leg3_ang_offset), -sin(leg3_ang_offset), 0.0}, 
		{sin(leg3_ang_offset), cos(leg3_ang_offset), 0.0},
		{0.0, 0.0, 1.0}
	};

	float leg4_offset_rot[3][3] = {
		{cos(leg4_ang_offset), -sin(leg4_ang_offset), 0.0}, 
		{sin(leg4_ang_offset), cos(leg4_ang_offset), 0.0},
		{0.0, 0.0, 1.0}
	};

	float leg5_offset_rot[3][3] = {
		{cos(leg5_ang_offset), -sin(leg5_ang_offset), 0.0}, 
		{sin(leg5_ang_offset), cos(leg5_ang_offset), 0.0},
		{0.0, 0.0, 1.0}
	};

	float leg6_offset_rot[3][3] = {
		{cos(leg6_ang_offset), -sin(leg6_ang_offset), 0.0}, 
		{sin(leg6_ang_offset), cos(leg6_ang_offset), 0.0},
		{0.0, 0.0, 1.0}
	};

	/// translation from O to leg frame ///
	float trans_O_to_leg1[3] = {-C1, 0.0, 0.0};
	float trans_O_to_leg2[3] = {-C2, 0.0, 0.0};
	float trans_O_to_leg3[3] = {-C2, 0.0, 0.0};
	float trans_O_to_leg4[3] = {-C1, 0.0, 0.0};
	float trans_O_to_leg5[3] = {-C2, 0.0, 0.0};
	float trans_O_to_leg6[3] = {-C2, 0.0, 0.0};

	/// XYZ home position of foot in O-frame ///
	float XYZ_home_OI[3] = {X_home+C1, 0, Z_home};
	float XYZ_home_OJ[3] = {(X_home+C2)*cos(beta*DEG2RAD), (X_home+C2)*sin(beta*DEG2RAD), Z_home};
	float XYZ_home_OK[3] = {-(X_home+C2)*cos(beta*DEG2RAD), (X_home+C2)*sin(beta*DEG2RAD), Z_home};
	float XYZ_home_OL[3] = {-(X_home+C1), 0, Z_home};
	float XYZ_home_OM[3] = {-(X_home+C2)*cos(beta*DEG2RAD), -(X_home+C2)*sin(beta*DEG2RAD), Z_home};
	float XYZ_home_ON[3] = {(X_home+C2)*cos(beta*DEG2RAD), -(X_home+C2)*sin(beta*DEG2RAD), Z_home};

	///////////////////////////////
	/// Normal walking steering ///
	///////////////////////////////
	float curve_path = T;
	float R_icc_max = 3000.0;
	float R_icc_min = 600.0;


public: 
	HexapodLib();

	///////////////////
	/// Math helper ///
	///////////////////
	void linspace(float Start, float Stop, int _size, float* Arr);

	void concat_arrays(float* arr1, float* arr2, int arr1_len, int arr2_len, float* arr3);

	float map_with_limit(float val, float in_min, float in_max, float out_min, float out_max);

	void matmul3_3(float A[3][3], float B[3][3], float C[3][3]);

	void matmul3_1(float A[3][3], float B[3], float C[3]);

	//////////////////////////
	/// Inverse Kinematics ///
	//////////////////////////
	void fwd(float theta1, float theta2, float theta3, float& x, float& y, float& z);

	void inv(float x, float y, float z, float& theta1, float& theta2, float& theta3);

	void inv_arrays(float* X, float* Y, float* Z, int len, float* THETA1, float* THETA2, float* THETA3);

	void inv_arrays_RAD(float* x_arr, float* y_arr, float* z_arr, int len, float* THE1, float* THE2, float* THE3);

	void invKinArray_to_ThetaArray(float XYZ[DATA_POINT_ALL][3], float* THE1, float* THE2, float* THE3);

	void invKinArray_to_ThetaDegArray(float XYZ[DATA_POINT_ALL][3], float* THE1, float* THE2, float* THE3);

	int kinAngle_To_servoPwm(int servo_id, float theta);

	void kinAngleArray_To_servoPwmArray(int servo_id, float* THETA, int len, int* PWM);

	////////////////////
	/// Crab-Walking ///
	////////////////////
	void XYZ_gen(int leg_id, float dir_rot_ang, float* X, float* Y, float* Z);

	void generate_crabWalkingLUT();

	///////////////////////////////
	/// Body translate & rotate ///
	///////////////////////////////
	void bodyRotate_to_newLegXYZ(float r, float p, float y, float* leg1_XYZ, float* leg2_XYZ, float* leg3_XYZ, float* leg4_XYZ, float* leg5_XYZ, float* leg6_XYZ);
	void bodyTranslate_to_newLegXYZ(float x, float y, float z, float* leg1_XYZ, float* leg2_XYZ, float* leg3_XYZ, float* leg4_XYZ, float* leg5_XYZ, float* leg6_XYZ);
	
	////////////////////////
	/// In-place turning ///
	////////////////////////
	void generate_inplaceTurning(float turn_deg, float* x_IKM_combine, float* y_IKM_combine, float* z_IKM_combine, float* x_JLN_combine, float* y_JLN_combine, float* z_JLN_combine);

	void generate_inplaceTurning_LUT();

	//////////////////////
	/// Normal walking ///
	//////////////////////
	void generate_steering_curve(int leg_no, int str_sign, float R_icc, float leg_XYZ[DATA_POINT_ALL][3]);
	void generate_normalWalking_LUT();

	// Futaba A700
	float pwm1_min = 800.0;
	float pwm1_mid = 1350.0;
	float pwm1_max = 1900.0;

	float joint1_min = -90.0;
	float joint1_max = 90.0;

	// DS3225
	float pwm2_min = 800.0;
	float pwm2_mid = 1500.0;
	float pwm2_max = 2200.0;

	float joint2_min = -70.0;
	float joint2_max = 70.0;

	// DS3225
	float pwm3_min = 800.0;
	float pwm3_mid = 1500.0;
	float pwm3_max = 2200.0;

	float joint3_min = -160.0;
	float joint3_max = -20.0;

	/// Home ///
	int PWM1_home;
	int PWM2_home;
	int PWM3_home;

	////////////////////////
	/// Crab-walking LUT ///
	///////////////////////
	/*
		12 is 360/30 so we have an increment of 30deg as 12 step resolution
		18 is the PWM1_1, PWM2_1, PWM3_1, ...., PWM1_6, PWM2_6, PWM3_6 total as 18 array of PWM
	*/
	int cw_ang_resolution = 30;
	int cw_array_length = int(360/cw_ang_resolution);
	int crab_walking_LUT[12][18][DATA_POINT_ALL];

	////////////////////////
	/// In-place turning ///
	////////////////////////

	// float inplace_turning_THETA_LUT[2][18][DATA_POINT_TURN_ALL];
	int inplace_turning_PWM_LUT[2][18][DATA_POINT_TURN_ALL];

	//////////////////////
	/// Normal walking ///
	//////////////////////
	// float normalWalking_THETA_LUT[15][18][DATA_POINT_ALL];
	int normalWalking_PWM_LUT[15][18][DATA_POINT_ALL];

};

#endif