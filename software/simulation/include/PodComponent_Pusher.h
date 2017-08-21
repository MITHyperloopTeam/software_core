#ifndef PUSHER_SYSTEM_H
#define PUSHER_SYSTEM_H

#include "PodComponent.h"
#include "Vec3d.h"
#include "yaml-cpp/yaml.h"
#include <lcm/lcm.h>
#include "lcmgen_c/mithl_sim_pusher_cmd_t.h"

class Pusher : public PodComponent {
  public:
    Pusher(YAML::Node config, lcm_t * lcm, double t_0);
    void reset(double t_0);
    void update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque);
    void output(double t);
    void simPusherCmdHandler(const mithl_sim_pusher_cmd_t * msg);
  private:
    lcm_t * lcm_;
    std::string channel_;
    double pushing_end_time_;
    double pushing_force_;

};

#endif