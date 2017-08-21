#include "PodComponent_Pusher.h"

#include "common_utils.h"

#include <iostream>

#include <random>
#include <algorithm>

static void _simPusherCmdHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_sim_pusher_cmd_t * msg, void * user){
  ((Pusher*)user)->simPusherCmdHandler(msg);
}

static inline double randRange(double low, double high){
  return ((high - low)*((double)rand() / double(RAND_MAX)) + low);
}

Pusher::Pusher(YAML::Node config, lcm_t * lcm, double t_0)
                              : PodComponent(config, t_0), lcm_(lcm)
{
  if (config["channel"])
    channel_ = config["channel"].as<std::string>();
  else
    channel_ = std::string("SIM_PUSHER_CMD"); 
  printf("Initializing a pusher system listening on %s\n", channel_.c_str());
  mithl_sim_pusher_cmd_t_subscribe(lcm_, channel_.c_str(), &_simPusherCmdHandler, this);
  reset(t_0);
}

void Pusher::reset(double t_0){
  last_update_time_ = t_0;
  pushing_end_time_ = t_0 - 1000.0;
  pushing_force_ = 0.0;
}

void Pusher::update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque)
{
  double dt = t - last_update_time_;
  last_update_time_ = t;

  if (t <= pushing_end_time_){
    force = Vec3d(pushing_force_, 0.0, 0.0);
  } else {
    force = Vec3d(0.0);
  }

  torque = Vec3d(0.0);
}

void Pusher::output(double t)
{
  // asdfjas
}

void Pusher::simPusherCmdHandler(const mithl_sim_pusher_cmd_t * msg){
  pushing_force_ = msg->force;
  pushing_end_time_ = getUnixTime() + msg->duration;
}
