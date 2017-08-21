#ifndef STATE_UTILS_H
#define STATE_UTILS_H

#define NUM_STATE_PARAMS 12

#include "Vec3d.h"
/*** Assuming state is a double[12] where:

x  \
y   \
z    | floating base
r    |
p   /
y  /
dx \
dy  \
dz   |  floating base
dr   |
dp  /
dy /

*/

Vec3d getPosition( double state[], int size = NUM_STATE_PARAMS);
Vec3d getVelocity( double state[], int size = NUM_STATE_PARAMS);
Vec3d getRpy( double state[], int size  = NUM_STATE_PARAMS);
Vec3d getAngVelocity( double state[], int size = NUM_STATE_PARAMS);

void setPosition( Vec3d& v, double state[], int size = NUM_STATE_PARAMS );
void setVelocity( Vec3d& v, double state[], int size = NUM_STATE_PARAMS );
void setRpy( Vec3d& v, double state[], int size = NUM_STATE_PARAMS );
void setAngVelocity( Vec3d& v, double state[], int size = NUM_STATE_PARAMS );

void setPosition( double v[3], double state[], int size = NUM_STATE_PARAMS );
void setVelocity( double v[3], double state[], int size = NUM_STATE_PARAMS );
void setRpy( double v[3], double state[], int size = NUM_STATE_PARAMS );
void setAngVelocity( double v[3], double state[], int size = NUM_STATE_PARAMS );


#endif