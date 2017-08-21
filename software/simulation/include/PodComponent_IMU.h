#ifndef IMU_H
#define IMU_H

#include "PodComponent.h"
#include "Vec3d.h"
#include "yaml-cpp/yaml.h"
#include <lcm/lcm.h>
#include "lcmgen_c/mithl_vectorXf_t.h"
#include "string.h"
#include <vector>

class IMU : public PodComponent {
  public:
    IMU(YAML::Node config, lcm_t * lcm, double t_0);
    void reset(double t_0);
    void update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque);
    void output(double t);
  private:
    lcm_t * lcm_;

    double last_broadcast_time_;
   	double broadcast_period_ = 0.01;
    double x_sign = 1.0;
    std::string channel;

   	Vec3d xyz_ddot_est_;

   	Vec3d last_rpy_dot_;
   	Vec3d last_xyz_dot_;
    std::vector<double> bias_;
    double stopped_noise_ = 0.001;
    double moving_noise_ = 0.1;
};

#endif