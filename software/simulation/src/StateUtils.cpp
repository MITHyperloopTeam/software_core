#include "StateUtils.h"

Vec3d getPosition( double state[], int size) {
  return Vec3d(state[0], state[1], state[2]);
}

Vec3d getRpy( double state[], int size) {
  return Vec3d(state[3], state[4], state[5]);
}

Vec3d getVelocity( double state[], int size) {
  return Vec3d(state[6], state[7], state[8]);
}

Vec3d getAngVelocity( double state[], int size) {
  return Vec3d(state[9], state[10], state[11]);
}

void setPosition( Vec3d& v, double state[], int size ) {
  state[0] = v.x();
  state[1] = v.y();
  state[2] = v.z();
}

void setRpy( Vec3d& v, double state[], int size ) {
  state[3] = v.x();
  state[4] = v.y();
  state[5] = v.z();
}

void setVel( Vec3d& v, double state[], int size ) {
  state[6] = v.x();
  state[7] = v.y();
  state[8] = v.z();
}

void setAngVelocity( Vec3d& v, double state[], int size ) {
  state[9] = v.x();
  state[10] = v.y();
  state[11] = v.z();
}

void setPosition( double v[3], double state[], int size ) {
  state[0] = v[0];
  state[1] = v[1];
  state[2] = v[2];
}

void setRpy( double v[3], double state[], int size ) {
  state[3] = v[0];
  state[4] = v[1];
  state[5] = v[2];
}

void setVel( double v[3], double state[], int size ) {
  state[6] = v[0];
  state[7] = v[1];
  state[8] = v[2];
}

void setAngVelocity( double v[3], double state[], int size ) {
  state[9] = v[0];
  state[10] = v[1];
  state[11] = v[2];
}