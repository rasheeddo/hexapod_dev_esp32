#include "driver/uart.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <hexapod_lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm_2 = Adafruit_PWMServoDriver(0x41);

HexapodLib h;

///////////////////////
/// SBUS PARSER ISR ///
///////////////////////
/// Parser Thread
#define UART_PORT_NUM UART_NUM_2
#define BUF_SIZE (1024 * 2)
#define RD_BUF_SIZE (1024)
static QueueHandle_t uart2_queue;
static const char * TAG = "";
#define U2RXD 16
#define U2TXD 17
uint8_t rxbuf[256];    
uint16_t rx_fifo_len;      
uint16_t ch[16];
uint16_t checksum = 0;

/// Main Thread
uint16_t sbus_ch[16];
uint16_t ch1;
uint16_t ch2;
uint16_t ch4;
uint16_t ch5;
uint16_t ch7;
uint16_t ch8;
int sbus_max = 1680;
int sbus_mid = 1024;
int sbus_min = 368;
int sbus_db_max = 1072;
int sbus_db_min = 976;

/////////////////
/// Hexapod  ///
////////////////
int delay_time = 25;
int walk_counter = 0;
int turn_counter = 0;
int ang_inc = 0;
int str_index = 7;
int turn_inc = 0;
unsigned long period = 0;
unsigned long last_stamp;
unsigned long last_home_stamp;
bool do_smooth = false;
bool stick_ch1_pressed = false;
bool stick_ch2_pressed = false;
bool stick_ch4_pressed = false;
bool stick_ch5_pressed = false;
bool sticks_pressed = false;
int max_delay = 50;
int min_delay = 10;
int default_delay = 25;
bool from_walk = false;
bool start_drive = false;

// mode //
char *robot_mode[] = {"NORM", "CRAB", "BODY"};
int robot_mode_index = 0;

// translation & rotatioon //
float x_trans;
float y_trans;
float z_trans;
float roll;
float pitch;
float yaw;
float leg1_XYZ[3];
float leg2_XYZ[3];
float leg3_XYZ[3];
float leg4_XYZ[3];
float leg5_XYZ[3];
float leg6_XYZ[3];

//////////////
/// BNO055 ///
//////////////
float qw;
float qx;
float qy;
float qz;
float r11, r12, r13, r21, r22, r23, r31, r32, r33;
float imu_roll, imu_pitch, body_yaw;
float comp_roll, comp_pitch;

/// PID ///
double Setpoint = 0.0;
double PIDRollInput, PIDRollOutput;
double PIDPitchInput, PIDPitchOutput;
double kp = 1.5; 
double ki = 3.0; 
double kd = 0.001;
double pid_out_min = -25.0;
double pid_out_max = 25.0;
PID bodyRollPID(&PIDRollInput, &PIDRollOutput, &Setpoint, kp, ki, kd, DIRECT);
PID bodyPitchPID(&PIDPitchInput, &PIDPitchOutput, &Setpoint, kp, ki, kd, DIRECT);

////////////////////////////
/// PWM Global variables ///
////////////////////////////
float smooth_gain = 0.65;

int pwm1_1;
int pwm2_1;
int pwm3_1;

int pwm1_2;
int pwm2_2;
int pwm3_2;

int pwm1_3;
int pwm2_3;
int pwm3_3;

int pwm1_4;
int pwm2_4;
int pwm3_4;

int pwm1_5;
int pwm2_5;
int pwm3_5;

int pwm1_6;
int pwm2_6;
int pwm3_6;

int pwm1_1_prev;
int pwm2_1_prev;
int pwm3_1_prev;

int pwm1_2_prev;
int pwm2_2_prev;
int pwm3_2_prev;

int pwm1_3_prev;
int pwm2_3_prev;
int pwm3_3_prev;

int pwm1_4_prev;
int pwm2_4_prev;
int pwm3_4_prev;

int pwm1_5_prev;
int pwm2_5_prev;
int pwm3_5_prev;

int pwm1_6_prev;
int pwm2_6_prev;
int pwm3_6_prev;


void setup() {
  
  Serial.begin(115200);
  
  /// BNO055 setup//
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  /// PID ///
  bodyRollPID.SetMode(AUTOMATIC);
  bodyRollPID.SetOutputLimits(pid_out_min, pid_out_max);
  bodyPitchPID.SetMode(AUTOMATIC);
  bodyPitchPID.SetOutputLimits(pid_out_min, pid_out_max);

  /// SBUS parser thread ///
  uart_config_t uart2_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_PORT_NUM, &uart2_config);
  esp_log_level_set(TAG, ESP_LOG_INFO);
  uart_set_pin(UART_PORT_NUM, U2TXD, U2RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_PORT_NUM, BUF_SIZE, BUF_SIZE, 20, &uart2_queue, 0);
  xTaskCreate(UART_ISR_ROUTINE, "UART_ISR_ROUTINE", 2048, NULL, 12, NULL);

  /// PCA9685 setup ///
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(5);

  pwm_2.begin();
  pwm_2.setOscillatorFrequency(27000000);
  pwm_2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(5);

  /// Hexapod setup ///
  h.generate_crabWalkingLUT();
  h.generate_inplaceTurning_LUT();
  h.generate_normalWalking_LUT();

  /// Init Home motion ///
  init_global_PWM();
  driveServo();
  last_stamp = millis();
  last_home_stamp = millis();
  delay(1000);
}

void loop() {

  // copy sbus values from "ch" to "sbus_ch"
  memcpy(sbus_ch, ch, sizeof(ch));

  // get data from BNO055 //
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

  imu_roll = atan2(r32, r33) * RAD2DEG;
  imu_pitch = asin(-r31) * RAD2DEG;
  body_yaw = atan2(-r21, r11) * RAD2DEG;

  // we're using 5 channels 
  ch1 = sbus_ch[0];
  ch2 = sbus_ch[1];
  ch4 = sbus_ch[3];
  ch5 = sbus_ch[4];
  ch7 = sbus_ch[6];
  ch8 = sbus_ch[7];

  // ch2 is mostly used for throttle speed 
  // the more it's pushed, the smaller the delay between movement in LUT
  if (ch2 > sbus_db_max){
    delay_time = map(ch2, sbus_mid, sbus_max, max_delay, min_delay);
  } else if (ch2 < sbus_db_min){
    delay_time = map(ch2, sbus_mid, sbus_min, max_delay, min_delay);
  } else {
    delay_time = default_delay;
  }

  // just to check if sticks got pushed or not
  stick_ch1_pressed = ((ch1 > sbus_db_max) || (ch1 < sbus_db_min));
  stick_ch2_pressed = ((ch2 > sbus_db_max) || (ch2 < sbus_db_min));
  stick_ch4_pressed = ((ch4 > sbus_db_max) || (ch4 < sbus_db_min));
  stick_ch5_pressed = ch5 < 1500;
  
  sticks_pressed =  (stick_ch1_pressed || stick_ch2_pressed || stick_ch4_pressed || stick_ch5_pressed);
                   
  //////////////////////
  /// Normal walking ///
  //////////////////////
  if (ch7 > 1500){
    robot_mode_index = 0;

    // steering only front //
    if (ch2 > sbus_db_max) {
      if ((ch1 < sbus_db_max) && (ch1 > 1000)) {
        str_index = 7;
      } else if (ch1 >= sbus_db_max) {
        str_index = (int)map(ch1, sbus_mid, sbus_max, 7, 14);
      } else if (ch1 <= sbus_db_min) {
        str_index = (int)map(ch1, sbus_min, sbus_mid, 0, 7);
      }
    } else {
      str_index = 7;
    }    
  } 

  //////////////////////////////////////////
  /// Crab-walking with in-place turning ///
  //////////////////////////////////////////
  else if (ch7 > 1000){
    robot_mode_index = 1;
    
    // crab-walking front // 
    if (ch2 > sbus_db_max){
      if (ch1 > sbus_db_max){
        ang_inc = (int)map(ch1, sbus_db_max, sbus_max, 11, 6);
      } else if (ch1 < sbus_db_min){
        ang_inc = (int)map(ch1, sbus_min, sbus_db_min, 6, 0);
      } else {
        ang_inc = 0;
      }
    } 
    // crab-walking back //
    else if (ch2 < sbus_db_min){
      if (ch1 > sbus_db_max){
        ang_inc = (int)map(ch1, sbus_db_max, sbus_max, 6, 11);
      } else if (ch1 < sbus_db_min){
        ang_inc = (int)map(ch1, sbus_min, sbus_db_min, 0, 6);
      } else {
        ang_inc = 6;
      }
    }
    // stay still //
    else {
      ang_inc = 0;
    }

    if (ch4 > sbus_db_max){
      turn_inc = 0;
    } else {
      turn_inc = 1;
    }
    
  }

  //////////////////////////////////////
  /// Rotation & Translation control ///
  //////////////////////////////////////
  else {
    robot_mode_index = 2;

    // translation //
    if (ch8 > 1500){
      x_trans = (float)map(ch4, sbus_min, sbus_max, -80.0, 80.0);
      y_trans = (float)map(ch2, sbus_min, sbus_max, -80.0, 80.0);
      z_trans = (float)map(ch1, sbus_min, sbus_max, -50.0, 50.0);
    } 
    // rotation //
    else {
      roll = (float)map(ch4, sbus_min, sbus_max, -15.0, 15.0);
      pitch = (float)map(ch2, sbus_min, sbus_max, -15.0, 15.0);
      yaw = (float)map(ch1, sbus_min, sbus_max, -15.0, 15.0);
    }


    
  }

  ////////////////////
  /// LUT counter ///
  //////////////////
  period = millis() - last_stamp;
  if ((period >= delay_time) && (sticks_pressed)) {

    if (start_drive){
      walk_counter = (int)DATA_POINT_ALL/4;
      start_drive = false;
    }

    /////////////////////////////////
    // normal-walking LUT counting //
    /////////////////////////////////
    if (robot_mode_index == 0){
      
      if (walk_counter < DATA_POINT_ALL){
        setNormalWalkingPWM(str_index, walk_counter);
        walk_counter++;
      } else {
        walk_counter = 0;
      }

      smooth_gain = 0.85;
      
    } 
    /////////////////////////////////////////
    // crab-walking & turning LUT counting //
    /////////////////////////////////////////
    else if (robot_mode_index == 1){

      // crab-walking //
      if (stick_ch2_pressed){
        if (walk_counter < DATA_POINT_ALL){
          setCrabWalkingPWM(ang_inc, walk_counter);
          walk_counter++;
        } else {
          walk_counter = 0;
        }
      }
      // turning //
      else if (stick_ch4_pressed){
        if (turn_counter < DATA_POINT_TURN_ALL){
          setInPlaceTurningPWM(turn_inc, turn_counter);
          turn_counter++;
        } else {
          turn_counter = 0;
        }
      }

      smooth_gain = 0.65;
      
    } 
    /////////////////////////////////
    // body trans-rot LUT counting //
    /////////////////////////////////
    else if (robot_mode_index == 2){

      if (ch8 > 1500){
        XYZ_to_PWM(x_trans, y_trans, z_trans);
      } else {
        if (ch5 > 1500){
          RPY_to_PWM(roll, pitch, yaw);
        } else{
          //comp_roll = -constrain(imu_pitch, -15.0, 15.0);
          //comp_pitch = constrain(imu_roll, -15.0, 15.0);
          PIDRollInput = -imu_pitch;
          PIDPitchInput = imu_roll;
          bodyRollPID.Compute();
          bodyPitchPID.Compute();
          
//          RPY_to_PWM(PIDRollOutput, 0.0, 0.0);
          RPY_to_PWM(PIDRollOutput, PIDPitchOutput, 0.0);
        }
        
      }
    }
    
    
//    if ((millis() - last_home_stamp) < 1500){
//      do_smooth = true;
//    } else {
//      do_smooth = false;
//    }

    do_smooth = true;
    from_walk = true;
    last_stamp = millis();
    
  } else if (sticks_pressed == false) {
    
    setHomePWM();
    from_walk = false;
    walk_counter = 0;
    turn_counter = 0;
    do_smooth = true;
    start_drive = true;
    last_home_stamp = millis();
    
    
  } else {

    // this is just a wait during next counter //
    // do nothing here //
    
  }

  ////////////////////
  /// Servo driving //
  //////////////////// 
  if (do_smooth == true) {

    pwm1_1 = smoothingPWM(smooth_gain, pwm1_1, pwm1_1_prev);
    pwm2_1 = smoothingPWM(smooth_gain, pwm2_1, pwm2_1_prev);
    pwm3_1 = smoothingPWM(smooth_gain, pwm3_1, pwm3_1_prev);

    pwm1_2 = smoothingPWM(smooth_gain, pwm1_2, pwm1_2_prev);
    pwm2_2 = smoothingPWM(smooth_gain, pwm2_2, pwm2_2_prev);
    pwm3_2 = smoothingPWM(smooth_gain, pwm3_2, pwm3_2_prev);

    pwm1_3 = smoothingPWM(smooth_gain, pwm1_3, pwm1_3_prev);
    pwm2_3 = smoothingPWM(smooth_gain, pwm2_3, pwm2_3_prev);
    pwm3_3 = smoothingPWM(smooth_gain, pwm3_3, pwm3_3_prev);

    pwm1_4 = smoothingPWM(smooth_gain, pwm1_4, pwm1_4_prev);
    pwm2_4 = smoothingPWM(smooth_gain, pwm2_4, pwm2_4_prev);
    pwm3_4 = smoothingPWM(smooth_gain, pwm3_4, pwm3_4_prev);

    pwm1_5 = smoothingPWM(smooth_gain, pwm1_5, pwm1_5_prev);
    pwm2_5 = smoothingPWM(smooth_gain, pwm2_5, pwm2_5_prev);
    pwm3_5 = smoothingPWM(smooth_gain, pwm3_5, pwm3_5_prev);

    pwm1_6 = smoothingPWM(smooth_gain, pwm1_6, pwm1_6_prev);
    pwm2_6 = smoothingPWM(smooth_gain, pwm2_6, pwm2_6_prev);
    pwm3_6 = smoothingPWM(smooth_gain, pwm3_6, pwm3_6_prev);
  }
  
  pwm1_1_prev = pwm1_1;
  pwm2_1_prev = pwm2_1;
  pwm3_1_prev = pwm3_1;

  pwm1_2_prev = pwm1_2;
  pwm2_2_prev = pwm2_2;
  pwm3_2_prev = pwm3_2;

  pwm1_3_prev = pwm1_3;
  pwm2_3_prev = pwm2_3;
  pwm3_3_prev = pwm3_3;

  pwm1_4_prev = pwm1_4;
  pwm2_4_prev = pwm2_4;
  pwm3_4_prev = pwm3_4;

  pwm1_5_prev = pwm1_5;
  pwm2_5_prev = pwm2_5;
  pwm3_5_prev = pwm3_5;

  pwm1_6_prev = pwm1_6;
  pwm2_6_prev = pwm2_6;
  pwm3_6_prev = pwm3_6;

  driveServo();

  /// Logging ///
  
  Serial.print("mode ");
  Serial.print(robot_mode[robot_mode_index]);
//  Serial.print(" ch1 ");
//  Serial.print(ch1);
//  Serial.print(" ch2 ");
//  Serial.print(ch2);
//  Serial.print(" ch4 ");
//  Serial.print(ch4);
//  Serial.print(" ch7 ");
//  Serial.print(ch7);
//  Serial.print(" ch5 ");
//  Serial.print(ch5);
  Serial.print(" smooth ");
  Serial.print(do_smooth);
  Serial.print(" smooth_K ");
  Serial.print(smooth_gain);
  Serial.print(" walk_counter ");
  Serial.print(walk_counter);
  Serial.print(" turn_counter ");
  Serial.print(turn_counter);
  Serial.print(" delay ");
  Serial.print(delay_time);
  Serial.print(" ang_in ");
  Serial.print(ang_inc);
  Serial.print(" turn_in ");
  Serial.print(turn_inc);
  Serial.print(" str_in ");
  Serial.print(str_index);
//  Serial.print(" r ");
//  Serial.print(roll);
//  Serial.print(" p ");
//  Serial.print(pitch);
//  Serial.print(" yw ");
//  Serial.print(yaw);
//  Serial.print("   imu_roll: ");
//  Serial.print(imu_roll, 2);
//  Serial.print(" imu_pitch: ");
//  Serial.print(imu_pitch, 2);
//  Serial.print(" pid_in_roll ");
//  Serial.print( PIDRollInput);
//  Serial.print(" pid_out_roll ");
//  Serial.print( PIDRollOutput);
//  Serial.print(" pid_in_pitch ");
//  Serial.print( PIDPitchInput);
//  Serial.print(" pid_out_pitch ");
//  Serial.println( PIDPitchOutput);
//  Serial.print(" body_yaw: ");
//  Serial.print(body_yaw, 2);
//  Serial.print("   comp_roll: ");
//  Serial.print(comp_roll, 2);
//  Serial.print(" comp_pitch: ");
//  Serial.println(comp_pitch, 2);
//  Serial.print(" x ");
//  Serial.print(x_trans);
//  Serial.print(" y ");
//  Serial.print(y_trans);
//  Serial.print(" z ");
//  Serial.print(z_trans);

  Serial.println(" ");

  
//  delay(1);
}

static void UART_ISR_ROUTINE(void *pvParameters)
{
  uart_event_t event;
  size_t buffered_size;
  bool exit_condition = false;

  //Infinite loop to run main bulk of task
  while (1) {

    //Loop will continually block (i.e. wait) on event messages from the event queue
    if (xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY)) {

      //Handle received event
      if (event.type == UART_DATA) {

        uint8_t UART2_data[128];
        //uint8_t buf[35];
        //uint16_t ch[16];
        int UART2_data_length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&UART2_data_length));
        UART2_data_length = uart_read_bytes(UART_NUM_2, UART2_data, UART2_data_length, 100);

        //Serial.println("LEN= ");Serial.println(UART2_data_length);

        //Serial.print("DATA= ");
        //for(byte i=0; i<UART2_data_length;i++) Serial.print(UART2_data[i]);
        //Serial.println("");

        for (int i = 0; i < 16; i++) {
          ch[i] = ((uint16_t)UART2_data[(2 * i) + 1] << 8 | ((uint16_t)UART2_data[(2 * i) + 2]));

          checksum ^= UART2_data[(2 * i) + 1];
          checksum ^= UART2_data[(2 * i) + 2];

        }

      }

      //Handle frame error event
      else if (event.type == UART_FRAME_ERR) {
        //TODO...
      }

      //Final else statement to act as a default case
      else {
        //TODO...
      }
    }

    //If you want to break out of the loop due to certain conditions, set exit condition to true
    if (exit_condition) {
      break;
    }
  }

  //Out side of loop now. Task needs to clean up and self terminate before returning
  vTaskDelete(NULL);
}

void driveServo() {
  pwm.writeMicroseconds(0, pwm1_1);
  pwm.writeMicroseconds(1, pwm2_1);
  pwm.writeMicroseconds(2, pwm3_1);

  pwm.writeMicroseconds(3, pwm1_2);
  pwm.writeMicroseconds(4, pwm2_2);
  pwm.writeMicroseconds(5, pwm3_2);

  pwm.writeMicroseconds(6, pwm1_3);
  pwm.writeMicroseconds(7, pwm2_3);
  pwm.writeMicroseconds(8, pwm3_3);

  pwm.writeMicroseconds(9, pwm1_4);
  pwm.writeMicroseconds(10, pwm2_4);
  pwm.writeMicroseconds(11, pwm3_4);

  pwm.writeMicroseconds(12, pwm1_5);
  pwm.writeMicroseconds(13, pwm2_5);
  pwm.writeMicroseconds(14, pwm3_5);

  pwm.writeMicroseconds(15, pwm1_6);
  pwm_2.writeMicroseconds(0, pwm2_6);
  pwm_2.writeMicroseconds(1, pwm3_6);
}

void setHomePWM() {
  pwm1_1 = h.PWM1_home;
  pwm2_1 = h.PWM2_home;
  pwm3_1 = h.PWM3_home;

  pwm1_2 = h.PWM1_home;
  pwm2_2 = h.PWM2_home;
  pwm3_2 = h.PWM3_home;

  pwm1_3 = h.PWM1_home;
  pwm2_3 = h.PWM2_home;
  pwm3_3 = h.PWM3_home;

  pwm1_4 = h.PWM1_home;
  pwm2_4 = h.PWM2_home;
  pwm3_4 = h.PWM3_home;

  pwm1_5 = h.PWM1_home;
  pwm2_5 = h.PWM2_home;
  pwm3_5 = h.PWM3_home;

  pwm1_6 = h.PWM1_home;
  pwm2_6 = h.PWM2_home;
  pwm3_6 = h.PWM3_home;
}

void init_global_PWM() {
  pwm1_1 = h.PWM1_home;
  pwm2_1 = h.PWM2_home;
  pwm3_1 = h.PWM3_home;

  pwm1_2 = h.PWM1_home;
  pwm2_2 = h.PWM2_home;
  pwm3_2 = h.PWM3_home;

  pwm1_3 = h.PWM1_home;
  pwm2_3 = h.PWM2_home;
  pwm3_3 = h.PWM3_home;

  pwm1_4 = h.PWM1_home;
  pwm2_4 = h.PWM2_home;
  pwm3_4 = h.PWM3_home;

  pwm1_5 = h.PWM1_home;
  pwm2_5 = h.PWM2_home;
  pwm3_5 = h.PWM3_home;

  pwm1_6 = h.PWM1_home;
  pwm2_6 = h.PWM2_home;
  pwm3_6 = h.PWM3_home;

  pwm1_1_prev = pwm1_1;
  pwm2_1_prev = pwm2_1;
  pwm3_1_prev = pwm3_1;

  pwm1_2_prev = pwm1_2;
  pwm2_2_prev = pwm2_2;
  pwm3_2_prev = pwm3_2;

  pwm1_3_prev = pwm1_3;
  pwm2_3_prev = pwm2_3;
  pwm3_3_prev = pwm3_3;

  pwm1_4_prev = pwm1_4;
  pwm2_4_prev = pwm2_4;
  pwm3_4_prev = pwm3_4;

  pwm1_5_prev = pwm1_5;
  pwm2_5_prev = pwm2_5;
  pwm3_5_prev = pwm3_5;

  pwm1_6_prev = pwm1_6;
  pwm2_6_prev = pwm2_6;
  pwm3_6_prev = pwm3_6;
}

int smoothingPWM(float smooth_gain, int new_pwm, int old_pwm) {
  int pwm_out;
  pwm_out = (int)((smooth_gain) * old_pwm + (1-smooth_gain) * new_pwm);

  return pwm_out;
}

void XYZ_to_PWM(float x_trans, float y_trans, float z_trans){

  float the1;
  float the2;
  float the3;

  h.bodyTranslate_to_newLegXYZ(x_trans, y_trans, z_trans, leg1_XYZ, leg2_XYZ, leg3_XYZ, leg4_XYZ, leg5_XYZ, leg6_XYZ);
  
  h.inv(leg1_XYZ[0], leg1_XYZ[1], leg1_XYZ[2], the1, the2, the3);
  pwm1_1 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_1 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_1 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);

  h.inv(leg2_XYZ[0], leg2_XYZ[1], leg2_XYZ[2], the1, the2, the3);
  pwm1_2 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_2 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_2 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
  
  h.inv(leg3_XYZ[0], leg3_XYZ[1], leg3_XYZ[2], the1, the2, the3);
  pwm1_3 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_3 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_3 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
  
  h.inv(leg4_XYZ[0], leg4_XYZ[1], leg4_XYZ[2], the1, the2, the3);
  pwm1_4 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_4 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_4 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
  
  h.inv(leg5_XYZ[0], leg5_XYZ[1], leg5_XYZ[2], the1, the2, the3);
  pwm1_5 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_5 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_5 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
  
  h.inv(leg6_XYZ[0], leg6_XYZ[1], leg6_XYZ[2], the1, the2, the3);
  pwm1_6 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_6 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_6 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
  
}

void RPY_to_PWM(float roll, float pitch, float yaw){
  
  float the1;
  float the2;
  float the3;
  
  h.bodyRotate_to_newLegXYZ(roll * DEG2RAD, pitch * DEG2RAD, yaw * DEG2RAD, leg1_XYZ, leg2_XYZ, leg3_XYZ, leg4_XYZ, leg5_XYZ, leg6_XYZ);
 
  h.inv(leg1_XYZ[0], leg1_XYZ[1], leg1_XYZ[2], the1, the2, the3);
  pwm1_1 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_1 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_1 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
  
  h.inv(leg2_XYZ[0], leg2_XYZ[1], leg2_XYZ[2], the1, the2, the3);
  pwm1_2 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_2 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_2 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
    
  h.inv(leg3_XYZ[0], leg3_XYZ[1], leg3_XYZ[2], the1, the2, the3);
  pwm1_3 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_3 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_3 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG); 
  
  h.inv(leg4_XYZ[0], leg4_XYZ[1], leg4_XYZ[2], the1, the2, the3);
  pwm1_4 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_4 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_4 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG); 
  
  h.inv(leg5_XYZ[0], leg5_XYZ[1], leg5_XYZ[2], the1, the2, the3);
  pwm1_5 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_5 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_5 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
   
  h.inv(leg6_XYZ[0], leg6_XYZ[1], leg6_XYZ[2], the1, the2, the3);
  pwm1_6 = h.kinAngle_To_servoPwm(0, the1 * RAD2DEG);
  pwm2_6 = h.kinAngle_To_servoPwm(1, the2 * RAD2DEG);
  pwm3_6 = h.kinAngle_To_servoPwm(2, the3 * RAD2DEG);
  
}

void setCrabWalkingPWM(int ang_inc, int counter) {
  pwm1_1 = h.crab_walking_LUT[ang_inc][0][counter];
  pwm2_1 = h.crab_walking_LUT[ang_inc][1][counter];
  pwm3_1 = h.crab_walking_LUT[ang_inc][2][counter];

  pwm1_2 = h.crab_walking_LUT[ang_inc][3][counter];
  pwm2_2 = h.crab_walking_LUT[ang_inc][4][counter];
  pwm3_2 = h.crab_walking_LUT[ang_inc][5][counter];

  pwm1_3 = h.crab_walking_LUT[ang_inc][6][counter];
  pwm2_3 = h.crab_walking_LUT[ang_inc][7][counter];
  pwm3_3 = h.crab_walking_LUT[ang_inc][8][counter];

  pwm1_4 = h.crab_walking_LUT[ang_inc][9][counter];
  pwm2_4 = h.crab_walking_LUT[ang_inc][10][counter];
  pwm3_4 = h.crab_walking_LUT[ang_inc][11][counter];

  pwm1_5 = h.crab_walking_LUT[ang_inc][12][counter];
  pwm2_5 = h.crab_walking_LUT[ang_inc][13][counter];
  pwm3_5 = h.crab_walking_LUT[ang_inc][14][counter];

  pwm1_6 = h.crab_walking_LUT[ang_inc][15][counter];
  pwm2_6 = h.crab_walking_LUT[ang_inc][16][counter];
  pwm3_6 = h.crab_walking_LUT[ang_inc][17][counter];
}

void setInPlaceTurningPWM(int ang_inc, int counter) {
  pwm1_1 = h.inplace_turning_PWM_LUT[ang_inc][0][counter];
  pwm2_1 = h.inplace_turning_PWM_LUT[ang_inc][1][counter];
  pwm3_1 = h.inplace_turning_PWM_LUT[ang_inc][2][counter];

  pwm1_2 = h.inplace_turning_PWM_LUT[ang_inc][3][counter];
  pwm2_2 = h.inplace_turning_PWM_LUT[ang_inc][4][counter];
  pwm3_2 = h.inplace_turning_PWM_LUT[ang_inc][5][counter];

  pwm1_3 = h.inplace_turning_PWM_LUT[ang_inc][6][counter];
  pwm2_3 = h.inplace_turning_PWM_LUT[ang_inc][7][counter];
  pwm3_3 = h.inplace_turning_PWM_LUT[ang_inc][8][counter];

  pwm1_4 = h.inplace_turning_PWM_LUT[ang_inc][9][counter];
  pwm2_4 = h.inplace_turning_PWM_LUT[ang_inc][10][counter];
  pwm3_4 = h.inplace_turning_PWM_LUT[ang_inc][11][counter];

  pwm1_5 = h.inplace_turning_PWM_LUT[ang_inc][12][counter];
  pwm2_5 = h.inplace_turning_PWM_LUT[ang_inc][13][counter];
  pwm3_5 = h.inplace_turning_PWM_LUT[ang_inc][14][counter];

  pwm1_6 = h.inplace_turning_PWM_LUT[ang_inc][15][counter];
  pwm2_6 = h.inplace_turning_PWM_LUT[ang_inc][16][counter];
  pwm3_6 = h.inplace_turning_PWM_LUT[ang_inc][17][counter];
}

void setNormalWalkingPWM(int str_index, int counter) {
  pwm1_1 = h.normalWalking_PWM_LUT[str_index][0][counter];
  pwm2_1 = h.normalWalking_PWM_LUT[str_index][1][counter];
  pwm3_1 = h.normalWalking_PWM_LUT[str_index][2][counter];

  pwm1_2 = h.normalWalking_PWM_LUT[str_index][3][counter];
  pwm2_2 = h.normalWalking_PWM_LUT[str_index][4][counter];
  pwm3_2 = h.normalWalking_PWM_LUT[str_index][5][counter];

  pwm1_3 = h.normalWalking_PWM_LUT[str_index][6][counter];
  pwm2_3 = h.normalWalking_PWM_LUT[str_index][7][counter];
  pwm3_3 = h.normalWalking_PWM_LUT[str_index][8][counter];

  pwm1_4 = h.normalWalking_PWM_LUT[str_index][9][counter];
  pwm2_4 = h.normalWalking_PWM_LUT[str_index][10][counter];
  pwm3_4 = h.normalWalking_PWM_LUT[str_index][11][counter];

  pwm1_5 = h.normalWalking_PWM_LUT[str_index][12][counter];
  pwm2_5 = h.normalWalking_PWM_LUT[str_index][13][counter];
  pwm3_5 = h.normalWalking_PWM_LUT[str_index][14][counter];

  pwm1_6 = h.normalWalking_PWM_LUT[str_index][15][counter];
  pwm2_6 = h.normalWalking_PWM_LUT[str_index][16][counter];
  pwm3_6 = h.normalWalking_PWM_LUT[str_index][17][counter];
}
