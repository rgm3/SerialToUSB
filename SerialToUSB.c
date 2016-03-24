/* 
 * Serial to USB HID
 */

// Libraries for register names (DDRD, PORTD) and the delay function
#include "SerialToUSB.h"

#define set_bit(address,bit) (address |= (1<<bit))
#define clear_bit(address,bit) (address &= ~(1<<bit))
#define toggle_bit(address,bit) (address ^= (1<<bit))
#define check_bit(address,bit) ((address & (1<<bit)) == (1<<bit))

#define LED 6 // PE6

#define USART_BAUDRATE 1200
//#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2) ) / ( USART_BAUDRATE )) - 1)
#define BAUD_PRESCALE 832

volatile char receivedByte = 0;

int main(void) {
  // The following line sets bit 5 high in register DDRD
  set_bit(DDRE, LED);    // PE6 as OUTPUT
  clear_bit(PORTE, LED); // LED off

  set_bit(DDRD, PD3); // PD3 is OUTPUT, TX
  clear_bit(DDRD, PD2); // PD2 is INPUT, RX

  _delay_ms(1000);


  cli();
  UCSR1B = (1 << RXEN1) | (1 << TXEN1); // Turn on the transmission and reception circuitry
  UCSR1C = (1 << UCSZ11) | (0 << UCSZ10); // Use 7-bit character sizes
  UBRR1H = (BAUD_PRESCALE >> 8);   // upper 8 bits of baud
  UBRR1L = (BAUD_PRESCALE & 0xff); // lower 8 bits

  UCSR1B |= (1 << RXCIE1 ); // enable receive complete interrupt
  sei(); // enable interrupts

  while (1) {
    // If a serial byte is ever received turn on light
    if(receivedByte != 0) {
      set_bit(PORTE, LED);
      _delay_ms(10);
      receivedByte = 0;
      clear_bit(PORTE, LED);
    }
  }

  return 0;
}

ISR(USART1_RX_vect) {
  receivedByte = UDR1 ; // Fetch the received byte value
  //UDR1 = receivedByte ; // Echo back the received byte back to the computer
}

/*
   s should consist of 3 bytes from the mouse
*/
void DecodeMouse(unsigned char *s,int *button,int *x,int *y)
{
   *button = 'n'; /* No button - should only happen on an error */
   if ((s[0] & 0x20) != 0)
      *button = 'l';
   else if ((s[0] & 0x10) != 0)
      *button = 'r';
   *x = (s[0] & 0x03) * 64 + (s[1] & 0x3F);
   if (*x > 127)
      *x = *x - 256;
   *y = (s[0] & 0x0C) * 16 + (s[2] & 0x3F);
   if (*y > 127)
      *y = *y - 256;
}
