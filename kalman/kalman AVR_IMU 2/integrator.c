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


#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "integrator.h"

void integrator_init(integrator *self, float normalize)
// Initialize the integrator structure.
{
    // Initialize the integrator.
    self->index = 0;
    self->normalize = normalize;
    self->values[0] = 0;
    self->values[1] = 0;
    self->values[2] = 0;
    self->values[3] = 0;
    self->integral = 0;
}


void integrator_add(integrator *self, float value)
// Add the value to the integrator structure.
{
    uint8_t i;
    float sum;

    // Initialize the index.
    i = self->index;

    // Store the normalized value.
    self->values[i] = value - self->normalize;

    // Peform the Runge-Kutta integrator algorithm.
    i = (i + 1) & 3;
    sum = self->values[i];
    i = (i + 1) & 3;
    sum += self->values[i] * 2.0;
    i = (i + 1) & 3;
    sum += self->values[i] * 2.0;
    i = (i + 1) & 3;
    sum += self->values[i];
    i = (i + 1) & 3;
    sum /= 6.0;

    // Update the index.
    self->index = i;

    // Update the integral.
    self->integral = self->integral + sum;
}


