/**
 * *************************************
 * Binary data TX
 *
 * Defs from:
 *    /usr/lib/avr/include/avr/iom328p.h
 * *************************************
 */
#define TX_BUFF_LEN 128
static uint8_t point = 0;
static uint8_t s[TX_BUFF_LEN];

void serialize16(int16_t a)
{
    s[point++]  = a;
    s[point++]  = a>>8&0xff;
      if(point >= TX_BUFF_LEN)
      {
#if WARNINGS
	  Serial.print("%%OO");
#endif
	  point--; // avoid breaking stuff
      }
}

void serialize8(uint8_t a)
{
    s[point++]  = a;
    if(point >= TX_BUFF_LEN)
    {
#if WARNINGS
	Serial.print("%%OO");
#endif
	point--; // avoid breaking stuff
    }
}

/** 
 * Adds data to the output buffer.
 * 
 * @param data 
 * @param size_bytes 
 */
void queue_raw_bin(void *data, int size_bytes)
{
    int i;
    for (i = 0 ; i < size_bytes ; i++)
	serialize8(*(((uint8_t*)data)+i));
}

// ***********************************
// Interrupt driven UART transmitter
// ***********************************
static uint8_t tx_ptr;
static uint8_t tx_busy = 0;

ISR(USART_TX_vect) {
    UDR0 = s[tx_ptr++];           /* Transmit next byte */
    if ( tx_ptr == point ) {      /* Check if all data is transmitted */
	UCSR0B &= ~(1<<TXCIE0);   /* Disable TX complete interrupt */
	tx_busy = 0;
	point = 0;                /* Clear TX buffer */
    }
}

/** 
 * Start of the data block transmission
 * Will print error if there was pending data in the queue.
 * 
 */
void UartSendData() {
#define SHOW_BYTE_COUNT 0
#if DEBUG && SHOW_BYTE_COUNT
    Serial.println();
    Serial.print(point,DEC);
    Serial.println();
#endif
    if(tx_busy)
    {
    	// Missed data!
#if WARNINGS
    	Serial.print("%%@@");
    	Serial.println(point-tx_ptr);
#endif
    }
    cli();
    tx_ptr = 0;
    UCSR0A |= (1<<TXC0); 
    UCSR0B |= (1<<TXCIE0); /* Enable TX complete interrupt */
    UDR0 = s[tx_ptr++];    /* Start transmission */
    tx_busy = 1;
    sei();
}
