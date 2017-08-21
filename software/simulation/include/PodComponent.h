#ifndef POD_COMPONENT_H
#define POD_COMPONENT_H

#include "StateUtils.h"
#include "Vec3d.h"
#include "yaml-cpp/yaml.h"

class PodComponent {
  

  public:
  	PodComponent(YAML::Node config, double t_0);
  	void reset(double t_0);
    virtual void update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque) = 0;
    virtual void output(double t) = 0;

    double last_update_time_;
    Vec3d xyz_;
    Vec3d rpy_;
};

#endif // POD_COMPONENT_H