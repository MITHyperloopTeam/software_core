#include <iostream>
#include <cmath>
#include <cstring>
#include "Vec3d.h"

using namespace std;

Vec3d::Vec3d( double d ) {
  elements[0] = d;
  elements[1] = d;
  elements[2] = d;
}

Vec3d::Vec3d( double x, double y, double z ) {
  elements[0] = x;
  elements[1] = y;
  elements[2] = z;
}

double Vec3d::x() const {
  return elements[0];
}

double Vec3d::y() const {
  return elements[1];
}

double Vec3d::z() const {
  return elements[2];
}


Vec3d Vec3d::transform(const Vec3d xyz, const Vec3d rpy) const {
  double w[3] = {elements[0], elements[1], elements[2]};
  double w_old[3] = {elements[0], elements[1], elements[2]};

  // applies rotation in yaw
  double yaw = rpy.z();
  w[0] = w_old[0]*cos(yaw) - w_old[1]*sin(yaw);
  w[1] = w_old[0]*sin(yaw) + w_old[1]*cos(yaw);
  w[2] =                                           w_old[2];
  memcpy(w_old, w, 3*sizeof(double));

  // applies rotation in pitch
  double pitch = rpy.y();
  w[0] = w_old[0]*cos(pitch)                + w_old[2]*sin(pitch);
  w[1] =                        w_old[1];
  w[2] = -w_old[0]*sin(pitch)               + w_old[2]*cos(pitch);
  memcpy(w_old, w, 3*sizeof(double));

  // applies rotation in roll
  double roll = rpy.x();
  w[0] = w_old[0];
  w[1] =              w_old[1]*cos(roll) - w_old[2]*sin(roll);
  w[2] =              w_old[1]*sin(roll) + w_old[2]*cos(roll);
  memcpy(w_old, w, 3*sizeof(double));


  // then finally translation and return
  return Vec3d(w[0] + xyz.x(), 
               w[1] + xyz.y(),
               w[2] + xyz.z());
}

Vec3d Vec3d::cross(Vec3d w) const {
  return Vec3d(
                 elements[1]*w.z() - elements[2]*w.y(),
                -elements[0]*w.z() + elements[2]*w.x(),
                 elements[0]*w.y() + elements[1]*w.x()
              );
}

Vec3d Vec3d::operator += (const Vec3d& v1) {
  elements[0] += v1.x();
  elements[1] += v1.y();
  elements[2] += v1.z();
  return *this;
}

// vector-vector compoonent-wise operations
Vec3d operator + ( const Vec3d& v1, const Vec3d& v2 ){
  return Vec3d( v1.x() + v2.x(), v1.y() + v2.y(), v1.z() + v2.z() );
}

Vec3d operator - ( const Vec3d& v1, const Vec3d& v2 ){
  return Vec3d( v1.x() - v2.x(), v1.y() - v2.y(), v1.z() - v2.z() );
}

Vec3d operator * ( const Vec3d& v1, const Vec3d& v2 ){
  return Vec3d( v1.x() * v2.x(), v1.y() * v2.y(), v1.z() * v2.z() );
}

// vector-scalar operations
Vec3d operator * ( const Vec3d& v1, const double d ) {
  return Vec3d( v1.x() * d, v1.y() * d, v1.z() * d);
}

Vec3d operator * ( const double d, const Vec3d& v1 ) {
  return Vec3d( v1.x() * d, v1.y() * d, v1.z() * d);
}

Vec3d operator / ( const Vec3d& v1, const double d  )  {
  return Vec3d( v1.x() / d, v1.y() / d, v1.z() / d);
}

// negate
Vec3d operator - ( const Vec3d& v ) {
  return Vec3d( -v.x(), -v.y(), -v.z() );
}

// print
std::ostream& operator << ( std::ostream& out, const Vec3d& v) {
  out << "<" << v.x() << ", " << v.y() << ", " << v.z() << ">";
  return out;
}