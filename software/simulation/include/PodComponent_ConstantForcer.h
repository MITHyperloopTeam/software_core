#ifndef CONSTANT_FORCER_H
#define CONSTANT_FORCER_H

#include "PodComponent.h"
#include "Vec3d.h"
#include "yaml-cpp/yaml.h"

class ConstantForcer : public PodComponent {
  public:
    ConstantForcer(YAML::Node config, double t_0);
    void update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque);
    void output(double t);
  private:
  	Vec3d force_;
  	Vec3d torque_;
};

#endif