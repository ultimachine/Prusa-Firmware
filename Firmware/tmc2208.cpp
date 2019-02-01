#include "Configuration.h"
#include "pins.h"

#ifdef TMC2208_COMMUNICATION

#include "Marlin.h"

#include <SoftwareSerialWithHalfDuplex.h>
#include <TMC2208Stepper_REGDEFS.h> //Conflicts with SoftwareSerialWithHalfDuplex multiple definition of __vector_9

#define TX_PIN X_UART_PIN //PCINT18

SoftwareSerialWithHalfDuplex mySerial(TX_PIN,TX_PIN,false,false); //RX, TX, Inverse Logic, Full Duplex

uint8_t calcCRC(uint8_t datagram[], uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t currentByte = datagram[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (currentByte & 0x01)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc = (crc << 1);
      }
      crc &= 0xff;
      currentByte = currentByte >> 1;
    } 
  }
  return crc;
}

void tmc2208_init() {
  #define BUF_SIZE 8
  char buf[BUF_SIZE];
  int buf_index=0;

  mySerial.begin(57600);

  uint8_t read_status[] = { TMC2208_SYNC, TMC2208_SLAVE_ADDR, REG_DRV_STATUS, 0x00, 0x0 }; // { 0x05, 0x00, 0x6F, 0x84 }

  MYSERIAL.print("CRC: ");
  MYSERIAL.println( calcCRC(read_status,3), HEX );

  read_status[3] = calcCRC(read_status,3);

  mySerial.write(read_status,4);

  for(int i=0; i < BUF_SIZE; i++) {
    while( !mySerial.available() ) { delayMicroseconds(1); }
    buf[i] = mySerial.read();
  }

  //Print response
  for(int i=0; i < BUF_SIZE; i++) {
    MYSERIAL.print("0x");
    MYSERIAL.print((uint8_t)buf[i],HEX);
    MYSERIAL.print(" ");
  }
  MYSERIAL.println();
}

void tmc2208_read_status() {}

#endif //TMC2208_COMMUNICATION





#ifdef TMC2208_COMMUNICATION__BROKEN

#include "Marlin.h"

#include <SoftwareSerialWithHalfDuplex.h>

//#include <TMC2208Stepper.h>
#include <TMC2208Stepper_REGDEFS.h> //Conflicts with SoftwareSerialWithHalfDuplex multiple definition of __vector_9
  //#define TMC2208_SYNC     0x05
  //#define TMC2208_SLAVE_ADDR  0x00
  //#define REG_DRV_STATUS     0x6F


#define DEBUG_TMC2208_UART

SoftwareSerialWithHalfDuplex x_uart(X_UART_PIN,X_UART_PIN,false,false); //Arguments: RX_PIN, TX_PIN, Inverse Logic, Full Duplex Mode
SoftwareSerialWithHalfDuplex y_uart(Y_UART_PIN,Y_UART_PIN,false,false);
SoftwareSerialWithHalfDuplex z_uart(Z_UART_PIN,Z_UART_PIN,false,false);
SoftwareSerialWithHalfDuplex e0_uart(E0_UART_PIN,E0_UART_PIN,false,false);

void tmc2208_init() {
  x_uart.begin(57600);
  y_uart.begin(57600);
  z_uart.begin(57600);
  e0_uart.begin(57600);
}

uint8_t calcCRC(uint8_t datagram[], uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t currentByte = datagram[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (currentByte & 0x01)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc = (crc << 1);
      }
      crc &= 0xff;
      currentByte = currentByte >> 1;
    } 
  }
  return crc;
}

#define BUF_SIZE 8
void tmc2208_read_status() {
  uint8_t read_status[] = { TMC2208_SYNC, TMC2208_SLAVE_ADDR, REG_DRV_STATUS, 0x00, 0x0 }; // { 0x05, 0x00, 0x6F, 0x84 }
  char buf[BUF_SIZE];
  int buf_index=0;

  #ifdef DEBUG_TMC2208_UART
    MYSERIAL.print("CRC: ");
    MYSERIAL.println( calcCRC(read_status,3), HEX );
  #endif

  x_uart.write(read_status,4);

  for(int i=0; i < BUF_SIZE; i++) {
    for(int timeout_counter=10; !(x_uart.available()) && (timeout_counter != 0); timeout_counter-- ) { delayMicroseconds(1); }
    buf[i] = x_uart.read();
  }

  //Print response
  #ifdef DEBUG_TMC2208_UART
    for(int i=0; i < BUF_SIZE; i++) {
      MYSERIAL.print("0x");
      MYSERIAL.print((uint8_t)buf[i],HEX);
      MYSERIAL.print(" ");
    }
    MYSERIAL.println();
  #endif
}

#endif //TMC2208_COMMUNICATION
