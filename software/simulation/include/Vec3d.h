#ifndef VECTOR_3D_H
#define VECTOR_3D_H

#ifndef COMPILE_FOR_ARDUINO
#include <iostream>
#endif

class Vec3d {
public:
  Vec3d( double x, double y, double z );
  Vec3d( double x = 0.0 );

  double x() const;
  double y() const;
  double z() const;

  Vec3d transform(const Vec3d xyz, const Vec3d rpy) const;
  Vec3d cross(const Vec3d xyz) const;

  Vec3d operator += (const Vec3d& v1);
  
private:
  double elements [3];
};

// vector-vector component-wise operations
Vec3d operator + ( const Vec3d& v1, const Vec3d& v2 );
Vec3d operator - ( const Vec3d& v1, const Vec3d& v2 );
Vec3d operator -= ( const Vec3d& v1, const Vec3d& v2 );
Vec3d operator * ( const Vec3d& v1, const Vec3d& v2 );

// vector-scalar operations
Vec3d operator * ( const Vec3d& v1, const double d );
Vec3d operator * ( const double d, const Vec3d& v1 );
Vec3d operator / ( const Vec3d& v1, const double d );

// negate
Vec3d operator - ( const Vec3d& v );

#ifndef COMPILE_FOR_ARDUINO
//print
std::ostream& operator << ( std::ostream& out, const Vec3d& v);
#endif

#endif // VECTOR_3D_H