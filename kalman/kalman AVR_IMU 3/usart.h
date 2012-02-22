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

#ifndef _IMU_USART_H_
#define _IMU_USART_H_ 1

// #define FOSC                8000000
// #define FOSC                16000000
#define FOSC                20000000

#if (FOSC == 8000000)
#define BAUD2UBRR_2400      416
#define BAUD2UBRR_4800      207
#define BAUD2UBRR_9600      103
#define BAUD2UBRR_14400      68
#define BAUD2UBRR_19200      51
#define BAUD2UBRR_28800      34
#define BAUD2UBRR_38400      25
#define BAUD2UBRR_57600      16
#define BAUD2UBRR_115200      8
#endif

#if (FOSC == 16000000)
#define BAUD2UBRR_2400      832
#define BAUD2UBRR_4800      416
#define BAUD2UBRR_9600      207
#define BAUD2UBRR_14400     138
#define BAUD2UBRR_19200     103
#define BAUD2UBRR_28800      68
#define BAUD2UBRR_38400      51
#define BAUD2UBRR_57600      34
#define BAUD2UBRR_115200     16
#endif

#if (FOSC == 20000000)
#define BAUD2UBRR_2400      1041
#define BAUD2UBRR_4800      520
#define BAUD2UBRR_9600      259
#define BAUD2UBRR_14400     173
#define BAUD2UBRR_19200     129
#define BAUD2UBRR_28800      86
#define BAUD2UBRR_38400      64
#define BAUD2UBRR_57600      42
#define BAUD2UBRR_115200     21
#endif

void usart_init(uint16_t ubrr);

uint8_t usart_xmit_ready(void);
uint8_t usart_recv_ready(void);

void usart_xmit(uint8_t data);
uint8_t usart_recv(void);

uint8_t usart_xmit_buffer_ready(void);
uint8_t usart_xmit_buffer(uint8_t* buffer, uint8_t buflen);
uint8_t usart_xmit_printf(const char *format, ...);

uint8_t usart_recv_buffer_has_eol(uint8_t eol);
uint8_t usart_recv_buffer(uint8_t* buffer, uint8_t buflen, uint8_t eol);

#endif // _IMU_USART_H_
