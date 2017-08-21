#include "math.h"
#include "arduino_matrix_math.h"
#include "string.h"

using namespace std;

namespace XenMatrix {


template <typename Scalar>
Vec3<Scalar>::Vec3( ) {
  elements[0] = 0.0;
  elements[1] = 0.0;
  elements[2] = 0.0;
}

template <typename Scalar>
Vec3<Scalar>::Vec3( Scalar * d ) {
  elements[0] = d[0];
  elements[1] = d[1];
  elements[2] = d[2];
}

template <typename Scalar>
Vec3<Scalar>::Vec3( Scalar x, Scalar y, Scalar z ) {
  elements[0] = x;
  elements[1] = y;
  elements[2] = z;
}

template <typename Scalar>
Scalar Vec3<Scalar>::x() const {
  return elements[0];
}

template <typename Scalar>
Scalar Vec3<Scalar>::y() const {
  return elements[1];
}

template <typename Scalar>
Scalar Vec3<Scalar>::z() const {
  return elements[2];
}

template <typename Scalar>
Mat3<Scalar> Vec3<Scalar>::outer(const Vec3<Scalar> xyz) const {
  Mat3<Scalar> out;
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      out(i, j) = elements[i]*xyz(j);
    }
  }
  return out;
}
template <typename Scalar>
Vec3<Scalar> Vec3<Scalar>::transform(const Vec3<Scalar> xyz, const Vec3<Scalar> rpy) const {
  Scalar w[3] = {elements[0], elements[1], elements[2]};
  Scalar w_old[3] = {elements[0], elements[1], elements[2]};

  // applies rotation in yaw
  Scalar yaw = rpy.z();
  w[0] = w_old[0]*cos(yaw) - w_old[1]*sin(yaw);
  w[1] = w_old[0]*sin(yaw) + w_old[1]*cos(yaw);
  w[2] =                                           w_old[2];
  memcpy(w_old, w, 3*sizeof(Scalar));

  // applies rotation in pitch
  Scalar pitch = rpy.y();
  w[0] = w_old[0]*cos(pitch)                + w_old[2]*sin(pitch);
  w[1] =                        w_old[1];
  w[2] = -w_old[0]*sin(pitch)               + w_old[2]*cos(pitch);
  memcpy(w_old, w, 3*sizeof(Scalar));

  // applies rotation in roll
  Scalar roll = rpy.x();
  w[0] = w_old[0];
  w[1] =              w_old[1]*cos(roll) - w_old[2]*sin(roll);
  w[2] =              w_old[1]*sin(roll) + w_old[2]*cos(roll);
  memcpy(w_old, w, 3*sizeof(Scalar));


  // then finally translation and return
  return Vec3<Scalar>(w[0] + xyz.x(), 
               w[1] + xyz.y(),
               w[2] + xyz.z());
}

template <typename Scalar>
Vec3<Scalar> Vec3<Scalar>::cross(Vec3<Scalar> w) const {
  return Vec3<Scalar>(
                 elements[1]*w.z() - elements[2]*w.y(),
                -elements[0]*w.z() + elements[2]*w.x(),
                 elements[0]*w.y() + elements[1]*w.x()
              );
}

template <typename Scalar>
Vec3<Scalar> Vec3<Scalar>::operator += (const Vec3<Scalar>& v1) {
  elements[0] += v1.x();
  elements[1] += v1.y();
  elements[2] += v1.z();
  return *this;
}



template <typename Scalar>
Mat3<Scalar>::Mat3( ) {
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
        elements[i][j] = 0.0;
    }
  }
}
/*
template <typename Scalar>
Mat3<Scalar>::Mat3( Scalar * x) {
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
        elements[i][j] = x[i*3 + j];
    }
  }
}*/
template <typename Scalar>
Mat3<Scalar>::Mat3( Scalar x[3][3]) {
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
        elements[i][j] = x[i][j];
    }
  }
}
template <typename Scalar>
Mat3<Scalar> Mat3<Scalar>::transpose() {
  Mat3<Scalar> ret;
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
        ret(i,j) = elements[j][i];
    }
  }
  return ret;
}
template <typename Scalar>
Mat3<Scalar> Mat3<Scalar>::operator += (const Mat3<Scalar>& v2) {
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
        elements[i][j] += v2.elements[i][j];
    }
  }
  return *this;
}


// must force instantiation
template class Vec3<double>;
template class Vec3<float>;
template class Mat3<double>;
template class Mat3<float>;
};
