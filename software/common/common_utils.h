#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H


#include <unistd.h>

// branchless sign!
// val = 0 -> 0
// val > 0 -> +1
// val < 0 -> -1
static int sgn(int val) {
    return (0 < val) - (val < 0);
}
static float sgnf(float val) {
    return (0. < val) - (val < 0.);
}
static double sgnd(double val) {
    return (0. < val) - (val < 0.);
}

#ifndef COMPILE_FOR_ARDUINO
static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}
#endif

#endif