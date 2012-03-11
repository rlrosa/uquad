/*
    Copyright (c) 2007 Michael P. Thompson <mpthompson@gmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"

#define XMIT_BUFFER_SIZE      64
#define RECV_BUFFER_SIZE      64

uint8_t xmit_buffer[XMIT_BUFFER_SIZE];
volatile uint8_t xmit_next;
volatile uint8_t xmit_count;

uint8_t recv_buffer[RECV_BUFFER_SIZE];
volatile uint8_t recv_buf_start;
volatile uint8_t recv_buf_end;

void usart_init(uint16_t ubrr)
{
    // Initialize the transmit buffer variables.
    xmit_next = 0;
    xmit_count = 0;

    // Initialize the receive buffer variables.
    recv_buf_start = 0;
    recv_buf_end = 0;

    // Set the baud rate.
    UBRR0 = ubrr;

    // Set transfer rate doubler.
    UCSR0A = (1<<U2X0);

    // Enable the receiver and transmitter.
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02);

    // Set frame format: 8 data, 1 stop bit.
    UCSR0C = (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00);

    // Enable the receive character interrupt.
    UCSR0B |= (1<<RXCIE0);
}


uint8_t usart_xmit_ready(void)
{
    // Return true if the transmit buffer is empty.
    return (UCSR0A & (1<<UDRE0)) ? 1 : 0;
}


uint8_t usart_recv_ready(void)
{
    // Return true if the receive buffer is full.
    return (UCSR0A & (1<<RXC0)) ? 1 : 0;
}


void usart_xmit(uint8_t data)
{
    // Wait for the transmit buffer to be empty.
    while (!(UCSR0A & (1<<UDRE0)));

    // Put data into the transmit buffer.
    UDR0 = data;
}


uint8_t usart_recv(void)
{
    // Wait for data to be received.
    while (!(UCSR0A & (1<<RXC0)));

    // Get the data from the receive buffer.
    return UDR0;

}


uint8_t usart_xmit_buffer_ready(void)
// Returns 1 if the xmit buffer is ready to send data otherwise returns 0.
{
    return (xmit_next < xmit_count) ? 0 : 1;
}


uint8_t usart_xmit_buffer(uint8_t* buffer, uint8_t buflen)
// Sends the buffer of data.  Returns 1 for success or 0 for failure.
{
    uint8_t i;

    // Sanity check the buffer length.
    if ((buflen == 0) || (buflen > XMIT_BUFFER_SIZE)) return 0;

    // Make sure we are ready to transmit.
    if (xmit_next != xmit_count) return 0;

    // Copy the buffer to the xmit buffer.
    for (i = 0; i < buflen; ++i) xmit_buffer[i] = buffer[i];

    // Set the transmit flags.
    xmit_next = 0;
    xmit_count = buflen;

    // Is the transmit buffer empty?
    if (UCSR0A & (1<<UDRE0))
    {
        // Send the first character.
        UDR0 = xmit_buffer[xmit_next++];
    }

    // Have we sent all characters?
    if (xmit_next < xmit_count)
    {
        // No. Interrupt when the USART transmit buffer is empty.
        UCSR0B |= (1<<UDRIE0);
    }

    // Return success.
    return 1;
}


uint8_t usart_xmit_printf(const char *format, ...)
{
    int buflen;
    va_list pvar;

    // Wait for buffer to be free.
    while (xmit_next != xmit_count);

    // Prepare to handle varargs.
    va_start(pvar, format);

    // Write the formatted string.
    buflen = vsnprintf(xmit_buffer, sizeof(xmit_buffer), format, pvar);

    // We are finished with varargs.
    va_end(pvar);

    // Set the transmit flags.
    xmit_next = 0;
    xmit_count = (uint8_t) buflen;

    // Is the transmit buffer empty?
    if (UCSR0A & (1<<UDRE0))
    {
        // Send the first character.
        UDR0 = xmit_buffer[xmit_next++];
    }

    // Have we sent all characters?
    if (xmit_next < xmit_count)
    {
        // No. Interrupt when the USART transmit buffer is empty.
        UCSR0B |= (1<<UDRIE0);
    }

    return 1;
}


SIGNAL(SIG_USART_DATA)
// Handles the data register empty interrupt.
{
    // Do we have another character to send?
    if (xmit_next < xmit_count)
    {
        // Send the next character.
        UDR0 = xmit_buffer[xmit_next++];
    }

    // Have we sent all characters?
    if (xmit_next >= xmit_count)
    {
        // Yes. Clear the USART transmit buffer is empty interrupt.
        UCSR0B &= ~(1<<UDRIE0);
    }
}


uint8_t usart_recv_buffer_has_eol(uint8_t eol)
// Returns 1 if the buffer contains an eol character otherwise zero.
{
    uint8_t i;
    uint8_t eol_found = 0;

    // Set the index at the start of the buffer.
    i = recv_buf_start;

    // Look for an eol character.
    while (!eol_found && (i != recv_buf_end))
    {
        // Did we find an eol character?
        eol_found = (recv_buffer[i] == eol) ? 1 : 0;

        // Increment the index.
        ++i;

        // Wrap around if needed.
        i &= (RECV_BUFFER_SIZE - 1);
    }

    return eol_found;
}


uint8_t usart_recv_buffer(uint8_t* buffer, uint8_t buflen, uint8_t eol)
// Reads the receive buffer and returns the length of the buffer read.
{
    uint8_t i;
    uint8_t count = 0;

    // Sanity check the buffer length.
    if ((buflen == 0) || (buflen > RECV_BUFFER_SIZE)) return 0;

    // Make sure we have data to read.
    if (recv_buf_start == recv_buf_end) return 0;

    // Clear interrupts.
    cli();

    // Read the buffer until it is filled, we hit the end of the receive
    // buffer or until we find and eol character.
    for (i = 0; (i < buflen) && (recv_buf_start != recv_buf_end); ++i)
    {
        // Get the next character.
        buffer[i] = recv_buffer[recv_buf_start];

        // Increment the count of characters read.
        ++count;

        // Increment the receive buffer start.
        ++recv_buf_start;

        // Wrap around if needed.
        recv_buf_start &= (RECV_BUFFER_SIZE - 1);

        // Stop if we hit an eol character.
        if (buffer[i] == eol) break;
    }

    // Enable interrupts.
    sei();

    return count;
}


SIGNAL(SIG_USART_RECV)
// Handles the data received interrupt.
{
    // Place the character into the recieve buffer.
    recv_buffer[recv_buf_end] = UDR0;

    // Increment the receive buffer end.
    ++recv_buf_end;

    // Wrap around if needed.
    recv_buf_end &= (RECV_BUFFER_SIZE - 1);

    // Have we overflowed the receive buffer?
    if (recv_buf_end == recv_buf_start)
    {
        // Increment the receive buffer start.
        ++recv_buf_start;

        // Wrap around if needed.
        recv_buf_start &= (RECV_BUFFER_SIZE - 1);
    }
}


