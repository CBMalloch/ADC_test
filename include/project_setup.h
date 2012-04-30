#ifndef __PROJECT_SETUP_H
#define __PROJECT_SETUP_H

#include <ADC_test_pin_assignments.h>

// System clock
#define FCY     40000000L
// FPB is the number of ticks of timer 1 (peripheral bus prescaled by 1) in 1 second
#define FPB     40000000L

extern volatile int programStatus;

#endif

