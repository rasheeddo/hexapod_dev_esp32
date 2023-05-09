#include "driver/uart.h"
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

int delay_time = 5;
unsigned long period = 0;
unsigned long last_stamp;
bool from_walk = false;
bool push_throttle = false;
unsigned long push_period = 0;
unsigned long last_push_thr_stamp = millis();
unsigned long throttle_trig = false;
bool do_smooth = false;

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


  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(5);

  pwm_2.begin();
  pwm_2.setOscillatorFrequency(27000000);
  pwm_2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(5);

  setHomePosition();
  init_global_PWM();
  last_stamp = millis();
  delay(3000);
}

void loop() {
//
//  memcpy(sbus_ch, ch, sizeof(ch));
//
//  period = millis() - last_stamp;
//  if ((period >= delay_time) && ((sbus_ch[0] < 900) || (sbus_ch[0] > 1070)) ) {
//
//    if (counter < DATA_POINT_TURN_ALL) {
//
//      if (sbus_ch[0] < 900) {
//        //turningInPlace(0, counter);
//        setInPlaceTurningPWM(0, counter);
//        counter++;
//      } else if (sbus_ch[0] > 1070) {
//        //turningInPlace(1, counter);
//        setInPlaceTurningPWM(1, counter);
//        counter++;
//      }
//
//
//    } else {
//
//      counter = 0;
//    }
//
//    last_stamp = millis();
//    from_walk = true;
//    do_smooth = true;
//
//  } else if ((sbus_ch[0] >= 900) && (sbus_ch[0] <= 1070)) {
//
//    //if (from_walk) {
//    //setHomePosition();
//    //}
//
//    setHomePWM();
//    counter = 0;
//    from_walk = false;
//    do_smooth = true;
//    delay(25);
//
//  } else {
//
//    // Do nothing...
//  }
//
//  // Smoothing PWM values
//  if (do_smooth == true) {
//    Serial.println("Do smoothing");
//    pwm1_1 = smoothingPWM(pwm1_1, pwm1_1_prev);
//    pwm2_1 = smoothingPWM(pwm2_1, pwm2_1_prev);
//    pwm3_1 = smoothingPWM(pwm3_1, pwm3_1_prev);
//
//    pwm1_2 = smoothingPWM(pwm1_2, pwm1_2_prev);
//    pwm2_2 = smoothingPWM(pwm2_2, pwm2_2_prev);
//    pwm3_2 = smoothingPWM(pwm3_2, pwm3_2_prev);
//
//    pwm1_3 = smoothingPWM(pwm1_3, pwm1_3_prev);
//    pwm2_3 = smoothingPWM(pwm2_3, pwm2_3_prev);
//    pwm3_3 = smoothingPWM(pwm3_3, pwm3_3_prev);
//
//    pwm1_4 = smoothingPWM(pwm1_4, pwm1_4_prev);
//    pwm2_4 = smoothingPWM(pwm2_4, pwm2_4_prev);
//    pwm3_4 = smoothingPWM(pwm3_4, pwm3_4_prev);
//
//    pwm1_5 = smoothingPWM(pwm1_5, pwm1_5_prev);
//    pwm2_5 = smoothingPWM(pwm2_5, pwm2_5_prev);
//    pwm3_5 = smoothingPWM(pwm3_5, pwm3_5_prev);
//
//    pwm1_6 = smoothingPWM(pwm1_6, pwm1_6_prev);
//    pwm2_6 = smoothingPWM(pwm2_6, pwm2_6_prev);
//    pwm3_6 = smoothingPWM(pwm3_6, pwm3_6_prev);
//  }
//  
//  pwm1_1_prev = pwm1_1;
//  pwm2_1_prev = pwm2_1;
//  pwm3_1_prev = pwm3_1;
//
//  pwm1_2_prev = pwm1_2;
//  pwm2_2_prev = pwm2_2;
//  pwm3_2_prev = pwm3_2;
//
//  pwm1_3_prev = pwm1_3;
//  pwm2_3_prev = pwm2_3;
//  pwm3_3_prev = pwm3_3;
//
//  pwm1_4_prev = pwm1_4;
//  pwm2_4_prev = pwm2_4;
//  pwm3_4_prev = pwm3_4;
//
//  pwm1_5_prev = pwm1_5;
//  pwm2_5_prev = pwm2_5;
//  pwm3_5_prev = pwm3_5;
//
//  pwm1_6_prev = pwm1_6;
//  pwm2_6_prev = pwm2_6;
//  pwm3_6_prev = pwm3_6;
//
//  driveServo();
//  Serial.print("counter ");
//  Serial.println(counter);
//  delay(1);


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

void init_global_PWM(){
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

int smoothingPWM(int new_pwm, int old_pwm) {
  int pwm_out;
  pwm_out = (0.8) * old_pwm + (0.2) * new_pwm;

  return pwm_out;
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

void turningInPlace(int ang_inc, int counter) {

  pwm.writeMicroseconds(0, h.inplace_turning_PWM_LUT[ang_inc][0][counter]);
  pwm.writeMicroseconds(1, h.inplace_turning_PWM_LUT[ang_inc][1][counter]);
  pwm.writeMicroseconds(2, h.inplace_turning_PWM_LUT[ang_inc][2][counter]);

  pwm.writeMicroseconds(3, h.inplace_turning_PWM_LUT[ang_inc][3][counter]);
  pwm.writeMicroseconds(4, h.inplace_turning_PWM_LUT[ang_inc][4][counter]);
  pwm.writeMicroseconds(5, h.inplace_turning_PWM_LUT[ang_inc][5][counter]);

  pwm.writeMicroseconds(6, h.inplace_turning_PWM_LUT[ang_inc][6][counter]);
  pwm.writeMicroseconds(7, h.inplace_turning_PWM_LUT[ang_inc][7][counter]);
  pwm.writeMicroseconds(8, h.inplace_turning_PWM_LUT[ang_inc][8][counter]);

  pwm.writeMicroseconds(9, h.inplace_turning_PWM_LUT[ang_inc][9][counter]);
  pwm.writeMicroseconds(10, h.inplace_turning_PWM_LUT[ang_inc][10][counter]);
  pwm.writeMicroseconds(11, h.inplace_turning_PWM_LUT[ang_inc][11][counter]);

  pwm.writeMicroseconds(12, h.inplace_turning_PWM_LUT[ang_inc][12][counter]);
  pwm.writeMicroseconds(13, h.inplace_turning_PWM_LUT[ang_inc][13][counter]);
  pwm.writeMicroseconds(14, h.inplace_turning_PWM_LUT[ang_inc][14][counter]);

  pwm.writeMicroseconds(15, h.inplace_turning_PWM_LUT[ang_inc][15][counter]);
  pwm_2.writeMicroseconds(0, h.inplace_turning_PWM_LUT[ang_inc][16][counter]);
  pwm_2.writeMicroseconds(1, h.inplace_turning_PWM_LUT[ang_inc][17][counter]);

}
