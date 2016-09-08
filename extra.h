#ifndef _EXTRA_H
#define _EXTRA_H

#include "Arduino.h"
#include "config.h"
#include "def.h"

/* Output.ino */
extern void writeMotors();
extern void initOutput();
extern void mixTable();

/* Serial.ino */
#define UART_NUMBER 2
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

extern volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
extern uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];

extern void UartSendData();
static void inline SerialEnd(uint8_t port) {
  switch (port) {
    case 1:
      UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1));
      break;
    default:
      /* error? */;
  }
}

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;  
  serialHeadRX[portnum] = h;
}

#endif	/* _EXTRA_H */
