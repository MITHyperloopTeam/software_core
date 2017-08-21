#ifndef ARDUINO_RANDOM_H
#define ARDUINO_RANDOM_H

// ref to http://forum.arduino.cc/index.php?topic=114504.0

#ifdef COMPILE_FOR_ARDUINO

#include "Arduino.h"
#define RAND_MAX 32767
int random();

float randZeroToOne();

#else

#include <stdlib.h>
#include <cmath>

float randZeroToOne();

#endif

/* generate a pair of normally distributed random numbers
 * using a Box-Muller transformation. The mean is 0 -- numbers
 * are equally likely to be + or -.  The required stdev is
 * given in 'sigma'.  Either result pointers can be NULL, if
 * you want just one number.
 */
void box_muller(float sigma, float *r1, float *r2);

float randNormal(float mean, float sigma);

#endif