#include <Arduino.h>
// #include "stm32f1xx_hal.h"
#include <pins.h>
#include <SPI.h>
#include <RH_NRF24.h>

#define Debug_serial Serial1

struct DataFrame{
  uint16_t ch1_x;
  uint16_t ch1_y;
  uint16_t ch2_x;
  uint16_t ch2_y;

  bool but1;
  bool but2;
  bool but3;
  bool but4;
};

DataFrame data_frame;
int16_t zero_ch1x;
int16_t zero_ch1y;
int16_t zero_ch2x;
int16_t zero_ch2y;


#include <RHSoftwareSPI.h>
RHSoftwareSPI spi;

RH_NRF24 radio(SPI2_CS, RADIO_EN, spi);

void initGPIO(void){
  pinMode(CH1_X, INPUT_ANALOG);
  pinMode(CH1_Y, INPUT_ANALOG);
  pinMode(CH2_X, INPUT_ANALOG);
  pinMode(CH2_Y, INPUT_ANALOG);

  pinMode(BUT1, INPUT_PULLUP);
  pinMode(BUT2, INPUT_PULLUP);
  pinMode(BUT3, INPUT_PULLUP);
  pinMode(BUT4, INPUT_PULLUP);
}

void calibrate_analog(void){
  // Beep the buzzer twice
  HardwareTimer tim1(1);
  pinMode(BUZ, PWM);
  tim1.setPeriod(1000);
  pwmWrite(BUZ, 32256);
  delay(100);
  pwmWrite(BUZ, 0);
  delay(100);
  pwmWrite(BUZ, 32256);
  delay(100);
  pwmWrite(BUZ, 0);
  delay(100);

  zero_ch1x = analogRead(CH1_X) - 2048;
  zero_ch1y = analogRead(CH1_Y) - 2048;
  zero_ch2x = analogRead(CH2_X) - 2048;
  zero_ch2y = analogRead(CH2_Y) - 2048;

  // Beep once done
  delay(500);
  pwmWrite(BUZ, 32256);
  delay(100);
  pwmWrite(BUZ, 0);

}

void measure_analog(void){
  data_frame.ch1_x = analogRead(CH1_X) - zero_ch1x;
  data_frame.ch1_y = analogRead(CH1_Y) - zero_ch1y;
  data_frame.ch2_x = analogRead(CH2_X) - zero_ch2x;
  data_frame.ch2_y = analogRead(CH2_Y) - zero_ch2y;
  char buf [50];
  sprintf(buf, "CH1 X: %i Y: %i -- CH2 X: %i, Y:%i\r\n", \
          data_frame.ch1_x, data_frame.ch1_y, data_frame.ch2_x, data_frame.ch2_y);
  Debug_serial.print(buf);
}

int median(int a[], int n){
  int temp, i, j;
  for (i = 0; i < n; i++){
    for (j = i + 1; j < n; j++){
      if (a[i] > a[j]){
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
      }
    }
  }
  return a[n/2];
}


void setup() {
  initGPIO();
  Debug_serial.begin(115200);
  spi.setPins(SPI2_MISO, SPI2_MOSI, SPI2_CLK);

  if (!radio.init())
    Debug_serial.println("init failed");
  if (!radio.setChannel(1))
    Debug_serial.println("setChannel failed");
  if (!radio.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm))
    Debug_serial.println("setRF failed");   

  // calibrate_analog(); 

}

void loop() {
  int oversampling = 10;
  int ch1_x_samples[oversampling];
  int ch1_y_samples[oversampling];
  int ch2_x_samples[oversampling];
  int ch2_y_samples[oversampling];
  int i;

  for (i = 0; i < oversampling; i++){
    ch1_x_samples[i] = analogRead(CH1_X) - zero_ch1x;
    ch1_y_samples[i] = analogRead(CH1_Y) - zero_ch1y;
    ch2_x_samples[i] = analogRead(CH2_X) - zero_ch2x;
    ch2_y_samples[i] = analogRead(CH2_Y) - zero_ch2y;
    delay(2);
  }
  
  data_frame.ch1_x = median(ch1_x_samples, oversampling);
  data_frame.ch1_y = median(ch1_y_samples, oversampling);
  data_frame.ch2_x = median(ch2_x_samples, oversampling);
  data_frame.ch2_y = median(ch2_y_samples, oversampling);


  // measure_analog();
  
  Debug_serial.println("Sending data");

  uint8_t *data = (uint8_t *)&data_frame;
  Debug_serial.print("Data length: ");
  Debug_serial.println(sizeof(data_frame));

  radio.send(data, sizeof(data_frame));

  radio.waitPacketSent();

  char buf [50];
  sprintf(buf, "CH1X: %02x %02x\ CH1Y: %02x %02x\nCH2X: %02x %02x CH2Y: %02x %02x\nBUT: %02x", \
          data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
  Debug_serial.println(buf);

}
