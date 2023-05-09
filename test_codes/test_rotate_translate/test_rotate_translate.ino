#include "driver/uart.h"
#include <ArduinoJson.h>
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


#define NUMERO_PORTA_SERIALE UART_NUM_2
#define BUF_SIZE (1024 * 2)
#define RD_BUF_SIZE (1024)
static QueueHandle_t uart2_queue;

static const char * TAG = "";

#define U2RXD 16
#define U2TXD 17

uint8_t rxbuf[256];     //Buffer di ricezione
uint16_t rx_fifo_len;        //Lunghezza dati


bool led_state = true;
uint16_t ch[16];
uint16_t checksum = 0;
uint16_t sbus_ch[16];

int delay_time = 25;
unsigned long period = 0;
unsigned long last_stamp;
bool from_walk = false;
bool push_throttle = false;
unsigned long push_period = 0;
unsigned long last_push_thr_stamp = millis();
unsigned long throttle_trig = false;
bool walk_mode;
float roll = 0.0 * DEG2RAD;
float pitch = 0.0 * DEG2RAD;
float yaw = 0.0 * DEG2RAD;
float x_trans = 0.0;
float y_trans = 0.0;
float z_trans = 0.0;
float leg1_XYZ[3];
float leg2_XYZ[3];
float leg3_XYZ[3];
float leg4_XYZ[3];
float leg5_XYZ[3];
float leg6_XYZ[3];

float leg1_the1;
float leg1_the2;
float leg1_the3;

float leg2_the1;
float leg2_the2;
float leg2_the3;

float leg3_the1;
float leg3_the2;
float leg3_the3;

float leg4_the1;
float leg4_the2;
float leg4_the3;

float leg5_the1;
float leg5_the2;
float leg5_the3;

float leg6_the1;
float leg6_the2;
float leg6_the3;

int leg1_PWM1;
int leg1_PWM2;
int leg1_PWM3;

int leg2_PWM1;
int leg2_PWM2;
int leg2_PWM3;

int leg3_PWM1;
int leg3_PWM2;
int leg3_PWM3;

int leg4_PWM1;
int leg4_PWM2;
int leg4_PWM3;

int leg5_PWM1;
int leg5_PWM2;
int leg5_PWM3;

int leg6_PWM1;
int leg6_PWM2;
int leg6_PWM3;

//// JSON ////
//const int capacity = JSON_ARRAY_SIZE(2) + 2*JSON_OBJECT_SIZE(2);
//StaticJsonDocument<capacity> doc;
//calculate capacity size by here https://arduinojson.org/v6/assistant/
StaticJsonDocument<256> doc;
JsonArray sbus = doc.createNestedArray("sbus");
JsonObject cmd_vel = doc.createNestedObject("cmd_vel");

void setup() {
  Serial.begin(115200);
  uart_config_t Configurazione_UART2 = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(NUMERO_PORTA_SERIALE, &Configurazione_UART2);

  esp_log_level_set(TAG, ESP_LOG_INFO);

  uart_set_pin(NUMERO_PORTA_SERIALE, U2TXD, U2RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  uart_driver_install(NUMERO_PORTA_SERIALE, BUF_SIZE, BUF_SIZE, 20, &uart2_queue, 0);

  xTaskCreate(UART_ISR_ROUTINE, "UART_ISR_ROUTINE", 2048, NULL, 12, NULL);

  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(5);

  pwm_2.begin();
  pwm_2.setOscillatorFrequency(27000000);
  pwm_2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(5);

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

  //  roll = 0.0 * DEG2RAD;
  //  pitch = 0.0 * DEG2RAD;
  //  yaw = 0.0 * DEG2RAD;
  //  h.bodyRotate_to_newLegXYZ(roll, pitch, yaw, leg1_XYZ, leg2_XYZ, leg3_XYZ, leg4_XYZ, leg5_XYZ, leg6_XYZ);
  x_trans = 0.0;
  y_trans = 0.0;
  z_trans = 0.0;
  h.bodyTranslate_to_newLegXYZ(x_trans, y_trans, z_trans, leg1_XYZ, leg2_XYZ, leg3_XYZ, leg4_XYZ, leg5_XYZ, leg6_XYZ);
  Serial.println(" ");
  Serial.print("leg1_XYZ ");
  print_array(leg1_XYZ, 3);
  Serial.print("leg2_XYZ ");
  print_array(leg2_XYZ, 3);
  Serial.print("leg3_XYZ ");
  print_array(leg3_XYZ, 3);
  Serial.print("leg4_XYZ ");
  print_array(leg4_XYZ, 3);
  Serial.print("leg5_XYZ ");
  print_array(leg5_XYZ, 3);
  Serial.print("leg6_XYZ ");
  print_array(leg6_XYZ, 3);
  Serial.println(" ");

  h.inv(leg1_XYZ[0], leg1_XYZ[1], leg1_XYZ[2], leg1_the1, leg1_the2, leg1_the3);
  h.inv(leg2_XYZ[0], leg2_XYZ[1], leg2_XYZ[2], leg2_the1, leg2_the2, leg2_the3);
  h.inv(leg3_XYZ[0], leg3_XYZ[1], leg3_XYZ[2], leg3_the1, leg3_the2, leg3_the3);
  h.inv(leg4_XYZ[0], leg4_XYZ[1], leg4_XYZ[2], leg4_the1, leg4_the2, leg4_the3);
  h.inv(leg5_XYZ[0], leg5_XYZ[1], leg5_XYZ[2], leg5_the1, leg5_the2, leg5_the3);
  h.inv(leg6_XYZ[0], leg6_XYZ[1], leg6_XYZ[2], leg6_the1, leg6_the2, leg6_the3);

  Serial.print("leg1 theta ");
  Serial.print(leg1_the1, 4);
  Serial.print(" ");
  Serial.print(leg1_the2, 4);
  Serial.print(" ");
  Serial.println(leg1_the3, 4);
  Serial.print("leg2 theta ");
  Serial.print(leg2_the1, 4);
  Serial.print(" ");
  Serial.print(leg2_the2, 4);
  Serial.print(" ");
  Serial.println(leg2_the3, 4);
  Serial.print("leg3 theta ");
  Serial.print(leg3_the1, 4);
  Serial.print(" ");
  Serial.print(leg3_the2, 4);
  Serial.print(" ");
  Serial.println(leg3_the3, 4);
  Serial.print("leg4 theta ");
  Serial.print(leg4_the1, 4);
  Serial.print(" ");
  Serial.print(leg4_the2, 4);
  Serial.print(" ");
  Serial.println(leg4_the3, 4);
  Serial.print("leg5 theta ");
  Serial.print(leg5_the1, 4);
  Serial.print(" ");
  Serial.print(leg5_the2, 4);
  Serial.print(" ");
  Serial.println(leg5_the3, 4);
  Serial.print("leg6 theta ");
  Serial.print(leg6_the1, 4);
  Serial.print(" ");
  Serial.print(leg6_the2, 4);
  Serial.print(" ");
  Serial.println(leg6_the3, 4);

  leg1_PWM1 = h.kinAngle_To_servoPwm(0, leg1_the1 * RAD2DEG);
  leg1_PWM2 = h.kinAngle_To_servoPwm(1, leg1_the2 * RAD2DEG);
  leg1_PWM3 = h.kinAngle_To_servoPwm(2, leg1_the3 * RAD2DEG);

  leg2_PWM1 = h.kinAngle_To_servoPwm(0, leg2_the1 * RAD2DEG);
  leg2_PWM2 = h.kinAngle_To_servoPwm(1, leg2_the2 * RAD2DEG);
  leg2_PWM3 = h.kinAngle_To_servoPwm(2, leg2_the3 * RAD2DEG);

  leg3_PWM1 = h.kinAngle_To_servoPwm(0, leg3_the1 * RAD2DEG);
  leg3_PWM2 = h.kinAngle_To_servoPwm(1, leg3_the2 * RAD2DEG);
  leg3_PWM3 = h.kinAngle_To_servoPwm(2, leg3_the3 * RAD2DEG);

  leg4_PWM1 = h.kinAngle_To_servoPwm(0, leg4_the1 * RAD2DEG);
  leg4_PWM2 = h.kinAngle_To_servoPwm(1, leg4_the2 * RAD2DEG);
  leg4_PWM3 = h.kinAngle_To_servoPwm(2, leg4_the3 * RAD2DEG);

  leg5_PWM1 = h.kinAngle_To_servoPwm(0, leg5_the1 * RAD2DEG);
  leg5_PWM2 = h.kinAngle_To_servoPwm(1, leg5_the2 * RAD2DEG);
  leg5_PWM3 = h.kinAngle_To_servoPwm(2, leg5_the3 * RAD2DEG);

  leg6_PWM1 = h.kinAngle_To_servoPwm(0, leg6_the1 * RAD2DEG);
  leg6_PWM2 = h.kinAngle_To_servoPwm(1, leg6_the2 * RAD2DEG);
  leg6_PWM3 = h.kinAngle_To_servoPwm(2, leg6_the3 * RAD2DEG);

  Serial.print("leg1 PWM ");
  Serial.print(leg1_PWM1);
  Serial.print(" ");
  Serial.print(leg1_PWM2);
  Serial.print(" ");
  Serial.println(leg1_PWM3);
  Serial.print("leg2 PWM ");
  Serial.print(leg2_PWM1);
  Serial.print(" ");
  Serial.print(leg2_PWM2);
  Serial.print(" ");
  Serial.println(leg2_PWM3);
  Serial.print("leg3 PWM ");
  Serial.print(leg3_PWM1);
  Serial.print(" ");
  Serial.print(leg3_PWM2);
  Serial.print(" ");
  Serial.println(leg3_PWM3);
  Serial.print("leg4 PWM ");
  Serial.print(leg4_PWM1);
  Serial.print(" ");
  Serial.print(leg4_PWM2);
  Serial.print(" ");
  Serial.println(leg4_PWM3);
  Serial.print("leg5 PWM ");
  Serial.print(leg5_PWM1);
  Serial.print(" ");
  Serial.print(leg5_PWM2);
  Serial.print(" ");
  Serial.println(leg5_PWM3);
  Serial.print("leg6 PWM ");
  Serial.print(leg6_PWM1);
  Serial.print(" ");
  Serial.print(leg6_PWM2);
  Serial.print(" ");
  Serial.println(leg6_PWM3);


  delay(3000);

  last_stamp = millis();

}

void loop() {

  memcpy(sbus_ch, ch, sizeof(ch));

  if (sbus_ch[6] > 1500) {
    delay_time = map(sbus_ch[1], 1070, 1680, 30, 5);
    if (sbus_ch[1] >= 1070) {

      if ((sbus_ch[0] < 1048) && (sbus_ch[0] > 1000)) {
        ang_inc = 0;
      } else if (sbus_ch[0] >= 1048) {
        ang_inc = (int)map(sbus_ch[0], 1048, 1680, 11, 6);
      } else if (sbus_ch[0] <= 1000) {
        ang_inc = (int)map(sbus_ch[0], 368, 980, 6, 0);
      }
      delay_time = map(sbus_ch[1], 1070, 1680, 30, 5);

    } else if (sbus_ch[1] < 980) {
      if ((sbus_ch[0] < 1048) && (sbus_ch[0] > 1000)) {
        ang_inc = 6;
      } else if (sbus_ch[0] >= 1048) {
        ang_inc = (int)map(sbus_ch[0], 1048, 1680, 6, 11);
      } else if (sbus_ch[0] <= 1000) {
        ang_inc = (int)map(sbus_ch[0], 368, 980, 0, 6);
      }

      delay_time = map(sbus_ch[1], 980, 368, 30, 5);

    } else {
      ang_inc = 0;
    }

  } else if (sbus_ch[6] > 900) {

    x_trans = (float)map(sbus_ch[3], 368, 1680, -80.0, 80.0);
    y_trans = (float)map(sbus_ch[1], 368, 1680, -80.0, 80.0);
    z_trans = (float)map(sbus_ch[0], 368, 1680, -50.0, 50.0);

  } else {

    roll = (float)map(sbus_ch[3], 368, 1680, -15.0, 15.0);
    pitch = (float)map(sbus_ch[1], 368, 1680, -15.0, 15.0);
    yaw = (float)map(sbus_ch[0], 368, 1680, -15.0, 15.0);

  }

  if (sbus_ch[6] > 1500) {
    Serial.print("sbus ch1 ");
    Serial.print(sbus_ch[0]);
    Serial.print(" sbus ch2 ");
    Serial.print(sbus_ch[1]);
    Serial.print(" ang_inc ");
    Serial.print(ang_inc);
    Serial.print(" counter ");
    Serial.print(counter);
    Serial.print(" delay time ");
    Serial.println(delay_time);
    
  } else if (sbus_ch[6] > 900) {
    
    Serial.print("sbus ch1 ");
    Serial.print(sbus_ch[0]);
    Serial.print(" sbus ch2 ");
    Serial.print(sbus_ch[1]);
    Serial.print(" sbus ch4 ");
    Serial.print(sbus_ch[3]);
    Serial.print(" x_trans ");
    Serial.print(x_trans);
    Serial.print(" y_trans ");
    Serial.print(y_trans);
    Serial.print(" z_trans ");
    Serial.print(z_trans);
    
  } else {
    
    Serial.print("sbus ch1 ");
    Serial.print(sbus_ch[0]);
    Serial.print(" sbus ch2 ");
    Serial.print(sbus_ch[1]);
    Serial.print(" sbus ch4 ");
    Serial.print(sbus_ch[3]);
    Serial.print(" roll ");
    Serial.print(roll);
    Serial.print(" pitch ");
    Serial.print(pitch);
    Serial.print(" yaw ");
    Serial.print(yaw);
  }


  if (sbus_ch[6] > 1500) {
    period = millis() - last_stamp;
    if ((period >= delay_time) && ((sbus_ch[1] >= 1070) || (sbus_ch[1] <= 980)) ) {

      if (counter < DATA_POINT_ALL) {

        crabWalking(ang_inc, counter);
        counter++;

      } else {

        counter = 0;
      }

      last_stamp = millis();
      from_walk = true;


    } else if ((sbus_ch[1] < 1070) && (sbus_ch[1] > 980)) {

      if (from_walk) {
        setHomePosition();
      }

      counter = 0;
      from_walk = false;
      delay(25);

    } else {
      // Do nothing...
    }
  } else if (sbus_ch[6] > 900) {
    h.bodyTranslate_to_newLegXYZ(x_trans, y_trans, z_trans, leg1_XYZ, leg2_XYZ, leg3_XYZ, leg4_XYZ, leg5_XYZ, leg6_XYZ);
    h.inv(leg1_XYZ[0], leg1_XYZ[1], leg1_XYZ[2], leg1_the1, leg1_the2, leg1_the3);
    h.inv(leg2_XYZ[0], leg2_XYZ[1], leg2_XYZ[2], leg2_the1, leg2_the2, leg2_the3);
    h.inv(leg3_XYZ[0], leg3_XYZ[1], leg3_XYZ[2], leg3_the1, leg3_the2, leg3_the3);
    h.inv(leg4_XYZ[0], leg4_XYZ[1], leg4_XYZ[2], leg4_the1, leg4_the2, leg4_the3);
    h.inv(leg5_XYZ[0], leg5_XYZ[1], leg5_XYZ[2], leg5_the1, leg5_the2, leg5_the3);
    h.inv(leg6_XYZ[0], leg6_XYZ[1], leg6_XYZ[2], leg6_the1, leg6_the2, leg6_the3);

    leg1_PWM1 = h.kinAngle_To_servoPwm(0, leg1_the1 * RAD2DEG);
    leg1_PWM2 = h.kinAngle_To_servoPwm(1, leg1_the2 * RAD2DEG);
    leg1_PWM3 = h.kinAngle_To_servoPwm(2, leg1_the3 * RAD2DEG);

    leg2_PWM1 = h.kinAngle_To_servoPwm(0, leg2_the1 * RAD2DEG);
    leg2_PWM2 = h.kinAngle_To_servoPwm(1, leg2_the2 * RAD2DEG);
    leg2_PWM3 = h.kinAngle_To_servoPwm(2, leg2_the3 * RAD2DEG);

    leg3_PWM1 = h.kinAngle_To_servoPwm(0, leg3_the1 * RAD2DEG);
    leg3_PWM2 = h.kinAngle_To_servoPwm(1, leg3_the2 * RAD2DEG);
    leg3_PWM3 = h.kinAngle_To_servoPwm(2, leg3_the3 * RAD2DEG);

    leg4_PWM1 = h.kinAngle_To_servoPwm(0, leg4_the1 * RAD2DEG);
    leg4_PWM2 = h.kinAngle_To_servoPwm(1, leg4_the2 * RAD2DEG);
    leg4_PWM3 = h.kinAngle_To_servoPwm(2, leg4_the3 * RAD2DEG);

    leg5_PWM1 = h.kinAngle_To_servoPwm(0, leg5_the1 * RAD2DEG);
    leg5_PWM2 = h.kinAngle_To_servoPwm(1, leg5_the2 * RAD2DEG);
    leg5_PWM3 = h.kinAngle_To_servoPwm(2, leg5_the3 * RAD2DEG);

    leg6_PWM1 = h.kinAngle_To_servoPwm(0, leg6_the1 * RAD2DEG);
    leg6_PWM2 = h.kinAngle_To_servoPwm(1, leg6_the2 * RAD2DEG);
    leg6_PWM3 = h.kinAngle_To_servoPwm(2, leg6_the3 * RAD2DEG);

    setBody();
  } else {
    h.bodyRotate_to_newLegXYZ(roll * DEG2RAD, pitch * DEG2RAD, yaw * DEG2RAD, leg1_XYZ, leg2_XYZ, leg3_XYZ, leg4_XYZ, leg5_XYZ, leg6_XYZ);
    h.inv(leg1_XYZ[0], leg1_XYZ[1], leg1_XYZ[2], leg1_the1, leg1_the2, leg1_the3);
    h.inv(leg2_XYZ[0], leg2_XYZ[1], leg2_XYZ[2], leg2_the1, leg2_the2, leg2_the3);
    h.inv(leg3_XYZ[0], leg3_XYZ[1], leg3_XYZ[2], leg3_the1, leg3_the2, leg3_the3);
    h.inv(leg4_XYZ[0], leg4_XYZ[1], leg4_XYZ[2], leg4_the1, leg4_the2, leg4_the3);
    h.inv(leg5_XYZ[0], leg5_XYZ[1], leg5_XYZ[2], leg5_the1, leg5_the2, leg5_the3);
    h.inv(leg6_XYZ[0], leg6_XYZ[1], leg6_XYZ[2], leg6_the1, leg6_the2, leg6_the3);

    leg1_PWM1 = h.kinAngle_To_servoPwm(0, leg1_the1 * RAD2DEG);
    leg1_PWM2 = h.kinAngle_To_servoPwm(1, leg1_the2 * RAD2DEG);
    leg1_PWM3 = h.kinAngle_To_servoPwm(2, leg1_the3 * RAD2DEG);

    leg2_PWM1 = h.kinAngle_To_servoPwm(0, leg2_the1 * RAD2DEG);
    leg2_PWM2 = h.kinAngle_To_servoPwm(1, leg2_the2 * RAD2DEG);
    leg2_PWM3 = h.kinAngle_To_servoPwm(2, leg2_the3 * RAD2DEG);

    leg3_PWM1 = h.kinAngle_To_servoPwm(0, leg3_the1 * RAD2DEG);
    leg3_PWM2 = h.kinAngle_To_servoPwm(1, leg3_the2 * RAD2DEG);
    leg3_PWM3 = h.kinAngle_To_servoPwm(2, leg3_the3 * RAD2DEG);

    leg4_PWM1 = h.kinAngle_To_servoPwm(0, leg4_the1 * RAD2DEG);
    leg4_PWM2 = h.kinAngle_To_servoPwm(1, leg4_the2 * RAD2DEG);
    leg4_PWM3 = h.kinAngle_To_servoPwm(2, leg4_the3 * RAD2DEG);

    leg5_PWM1 = h.kinAngle_To_servoPwm(0, leg5_the1 * RAD2DEG);
    leg5_PWM2 = h.kinAngle_To_servoPwm(1, leg5_the2 * RAD2DEG);
    leg5_PWM3 = h.kinAngle_To_servoPwm(2, leg5_the3 * RAD2DEG);

    leg6_PWM1 = h.kinAngle_To_servoPwm(0, leg6_the1 * RAD2DEG);
    leg6_PWM2 = h.kinAngle_To_servoPwm(1, leg6_the2 * RAD2DEG);
    leg6_PWM3 = h.kinAngle_To_servoPwm(2, leg6_the3 * RAD2DEG);

    setBody();
  }

  Serial.println();
  delay(1);
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

void setBody() {

  pwm.writeMicroseconds(0, leg1_PWM1);
  pwm.writeMicroseconds(1, leg1_PWM2);
  pwm.writeMicroseconds(2, leg1_PWM3);

  pwm.writeMicroseconds(3, leg2_PWM1);
  pwm.writeMicroseconds(4, leg2_PWM2);
  pwm.writeMicroseconds(5, leg2_PWM3);

  pwm.writeMicroseconds(6, leg3_PWM1);
  pwm.writeMicroseconds(7, leg3_PWM2);
  pwm.writeMicroseconds(8, leg3_PWM3);

  pwm.writeMicroseconds(9, leg4_PWM1);
  pwm.writeMicroseconds(10, leg4_PWM2);
  pwm.writeMicroseconds(11, leg4_PWM3);

  pwm.writeMicroseconds(12, leg5_PWM1);
  pwm.writeMicroseconds(13, leg5_PWM2);
  pwm.writeMicroseconds(14, leg5_PWM3);

  pwm.writeMicroseconds(15, leg6_PWM1);
  pwm_2.writeMicroseconds(0, leg6_PWM2);
  pwm_2.writeMicroseconds(1, leg6_PWM3);
}

void crabWalking(int ang_inc, int counter) {

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

}

void setHomePosition() {

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
