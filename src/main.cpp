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

void measure_analog(void){
  data_frame.ch1_x = analogRead(CH1_X);
  data_frame.ch1_y = analogRead(CH1_Y);
  data_frame.ch2_x = analogRead(CH2_X);
  data_frame.ch2_y = analogRead(CH2_Y);
  char buf [50];
  sprintf(buf, "CH1 X: %i Y: %i -- CH2 X: %i, Y:%i\r\n", \
          data_frame.ch1_x, data_frame.ch1_y, data_frame.ch2_x, data_frame.ch2_y);
  Debug_serial.print(buf);
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

}

void loop() {
  delay(500);
  measure_analog();
  
  // Set data_frame to dummy data
  // data_frame.ch1_x = 255;
  // data_frame.ch1_y = 0;
  // data_frame.ch2_x = 127;
  // data_frame.ch2_y = 256;
  // data_frame.but1 = 0;
  // data_frame.but2 = 1;
  // data_frame.but3 = 0;
  // data_frame.but4 = 1;

  Debug_serial.println("Sending to nrf24_server");
  // Send a message to nrf24_server
  // uint8_t data[] = "Hello World!";
  uint8_t *data = (uint8_t *)&data_frame;
  Debug_serial.print("Data length: ");
  Debug_serial.println(sizeof(data_frame));

  radio.send(data, sizeof(data_frame));

  radio.waitPacketSent();

  char buf [50];
  sprintf(buf, "CH1X: %02x %02x\ CH1Y: %02x %02x\nCH2X: %02x %02x CH2Y: %02x %02x\nBUT: %02x", \
          data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
  Debug_serial.println(buf);

  // // Now wait for a reply
  // uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  // uint8_t len = sizeof(buf);
  // if (radio.waitAvailableTimeout(500))
  // { 
  //   // Should be a reply message for us now   
  //   if (radio.recv(buf, &len))
  //   {
  //     Debug_serial.print("got reply: ");
  //     Debug_serial.println((char*)buf);
  //   }
  //   else
  //   {
  //     Debug_serial.println("recv failed");
  //   }
  // }
  // else
  // {
  //   Debug_serial.println("No reply, is nrf24_server running?");
  // }
}
