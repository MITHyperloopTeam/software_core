#ifndef LEV_FORCER_H
#define LEV_FORCER_H

#include "PodComponent.h"
#include "Vec3d.h"
#include "yaml-cpp/yaml.h"

class LevForcer : public PodComponent {
  public:
    LevForcer(YAML::Node config, double t_0);
    void update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque);
    void output(double t);
    double get_drag_force(double velocity);
  private:
  	Vec3d force_;
  	Vec3d torque_;
    //double pod_mass_;
    //double podlev_slope_;
    //double podlev_height_;
    //double gravity_;
    double podlev_drag_peak_speed_;
    double podlev_drag_peak_magnitude_;
    double podlev_drag_peak_spread_;
    double podlev_drag_above_peak_;
    double podlev_drag_above_peak_slope_;

};

#endif