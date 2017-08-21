/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Cleaned up style by Greg Izatt in 2017.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 */
 
#ifndef XEN_MATRIX_H
#define XEN_MATRIX_H
 
namespace XenMatrix
{
  template <typename Scalar>
  class Mat3;

  template <typename Scalar>
  class Vec3 {
  public:
    Vec3( );
    Vec3( Scalar x, Scalar y, Scalar z );
    Vec3( Scalar * x);

    Scalar x() const;
    Scalar y() const;
    Scalar z() const;

    Vec3<Scalar> transform(const Vec3<Scalar> xyz, const Vec3<Scalar> rpy) const;
    Vec3<Scalar> cross(const Vec3<Scalar> xyz) const;
    Mat3<Scalar> outer(const Vec3<Scalar> xyz) const;
    Vec3 operator += (const Vec3<Scalar>& v1);
    Scalar operator ()(int i) const   { return elements[i]; }
    Scalar& operator ()(int i) { return elements[i]; }

  private:
    Scalar elements [3];

  };

  template <typename Scalar>
  class Mat3 {
  public:
    Mat3( );
    //Mat3( Scalar * x);
    Mat3( Scalar x[3][3]);
    Mat3 operator += (const Mat3<Scalar>& m1);
    Scalar operator ()(int i, int j) const { return elements[i][j]; }
    Scalar& operator ()(int i, int j) { return elements[i][j]; }
    Mat3 transpose();
    Scalar elements [3][3];
  };

  /*************** VECTOR OPERATIONS **************/
  // vector-vector compoonent-wise operations
  template <typename Scalar>
  Vec3<Scalar> operator + ( const Vec3<Scalar>& v1, const Vec3<Scalar>& v2 ){
    return Vec3<Scalar>( v1.x() + v2.x(), v1.y() + v2.y(), v1.z() + v2.z() );
  }

  template <typename Scalar>
  Vec3<Scalar> operator - ( const Vec3<Scalar>& v1, const Vec3<Scalar>& v2 ){
    return Vec3<Scalar>( v1.x() - v2.x(), v1.y() - v2.y(), v1.z() - v2.z() );
  }

  template <typename Scalar>
  Scalar operator * ( const Vec3<Scalar>& v1, const Vec3<Scalar>& v2 ){
    return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
  }

  // vector-scalar operations
  template <typename Scalar>
  Vec3<Scalar> operator + ( const Vec3<Scalar>& v1, const Scalar d ) {
    return Vec3<Scalar>( v1.x() + d, v1.y() + d, v1.z() + d);
  }

  template <typename Scalar>
  Vec3<Scalar> operator + ( const Scalar d, const Vec3<Scalar>& v1) {
    return Vec3<Scalar>( v1.x() + d, v1.y() + d, v1.z() + d);
  }

  template <typename Scalar>
  Vec3<Scalar> operator * ( const Vec3<Scalar>& v1, const Scalar d ) {
    return Vec3<Scalar>( v1.x() * d, v1.y() * d, v1.z() * d);
  }

  template <typename Scalar>
  Vec3<Scalar> operator * ( const Scalar d, const Vec3<Scalar>& v1 ) {
    return Vec3<Scalar>( v1.x() * d, v1.y() * d, v1.z() * d);
  }

  template <typename Scalar>
  Vec3<Scalar> operator / ( const Vec3<Scalar>& v1, const Scalar d  )  {
    return Vec3<Scalar>( v1.x() / d, v1.y() / d, v1.z() / d);
  }

  // negate
  template <typename Scalar>
  Vec3<Scalar> operator - ( const Vec3<Scalar>& v ) {
    return Vec3<Scalar>( -v.x(), -v.y(), -v.z() );
  }


  /*************** 3x3 MATRIX OPERATIONS **************/
  // matrix-matrix compoonent-wise operations
  template <typename Scalar>
  Mat3<Scalar> operator + ( const Mat3<Scalar>& v1, const Mat3<Scalar>& v2 ){
    Mat3<Scalar> out;
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) out(i, j) = v1(i, j) + v2(i, j);
    return out;
  }
  template <typename Scalar>
  Mat3<Scalar> operator - ( const Mat3<Scalar>& v1, const Mat3<Scalar>& v2 ){
    Mat3<Scalar> out(v1);
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) out(i, j) = v1(i, j) - v2(i, j);
    return out;
  }
  template <typename Scalar>
  Mat3<Scalar> operator * ( const Mat3<Scalar>& v1, const Mat3<Scalar>& v2 ){
    Mat3<Scalar> out;
    for (int i=0; i<3; i++) {
      for (int j=0; j<3; j++) {
        out(i, j) = v1(i,0) * v2(0,j) + v1(i,1) * v2(1,j) + v1(i,2) * v2(2,j);
      }
    }
    return out;
  }
  template <typename Scalar>
  Vec3<Scalar> operator * ( const Mat3<Scalar>& m, const Vec3<Scalar>& v ){
    Vec3<Scalar> out;
    for (int i=0; i<3; i++) {
      out(i) = m(i, 0) * v(0) + m(i, 1) * v(1) + m(i, 2) * v(2);
    }
    return out;
  }
  template <typename Scalar>
  Vec3<Scalar> operator * ( const Vec3<Scalar>& v, const Mat3<Scalar>& m ){
    Vec3<Scalar> out;
    for (int i=0; i<3; i++) {
      out(i) = m(0, i) * v(0) + m(1, i) * v(1) + m(2, i) * v(2);
    }
    return out;
  }

  template <typename Scalar>
  Mat3<Scalar> operator * ( const Mat3<Scalar>& v1, const Scalar d ) {
    Mat3<Scalar> out (v1);
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) out(i, j) = v1(i, j)*d;
    return out;
  }

  template <typename Scalar>
  Mat3<Scalar> operator + ( const Mat3<Scalar>& v1, const Scalar d ) {
    Mat3<Scalar> out (v1);
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) out(i, j) = v1(i, j) + d;
    return out;
  }

};

#endif