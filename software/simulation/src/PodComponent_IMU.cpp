#include "PodComponent_IMU.h"

#include "common_utils.h"

#include <iostream>

#include <random>
#include <algorithm>

static inline double randRange(double low, double high){
  return ((high - low)*((double)rand() / double(RAND_MAX)) + low);
}

IMU::IMU(YAML::Node config, lcm_t * lcm, double t_0)
                              : PodComponent(config, t_0), lcm_(lcm)
{
  printf("Initializing an IMU simulation\n");
  if (config["stopped_noise"])
    stopped_noise_ = config["stopped_noise"].as<double>();
  if (config["moving_noise"])
    moving_noise_ = config["moving_noise"].as<double>();
  if (config["broadcast_period"])
    broadcast_period_ = config["broadcast_period"].as<double>();
  if (config["channel"])
    channel = config["channel"].as<std::string>();
  if (config["x_flip"])
    x_sign = config["x_flip"].as<double>();

  if (config["bias"])
    bias_ = config["bias"].as<std::vector<double>>();
  else {
    bias_.resize(6);
    for (int i=0; i<6; i++) bias_[i] = 0.0;
  }

  printf("NOTICE: Other than xflip, this IMU does NOT know how to rotate.\n");
  printf("It's probably wrong about absolutely everything but that axis, TBH.\n");
  reset(t_0);
}

void IMU::reset(double t_0){
  last_update_time_ = t_0;
  last_broadcast_time_ = t_0;

  xyz_ddot_est_ = Vec3d(0.0);
  
  last_rpy_dot_ = Vec3d(0.0);
  last_xyz_dot_ = Vec3d(0.0);
}

void IMU::update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque)
{
  double dt = t - last_update_time_;
  last_update_time_ = t;

  xyz_ddot_est_ = (xyz_dot - last_xyz_dot_)/dt;
  xyz_ddot_est_ += Vec3d(0.0, 0.0, -9.81);

  last_rpy_dot_ = rpy_dot;
  last_xyz_dot_ = xyz_dot;

  force = Vec3d(0.0);
  torque = Vec3d(0.0);
}

void IMU::output(double t)
{
  if (t - last_broadcast_time_ >= broadcast_period_){
    last_broadcast_time_ = t;
    mithl_vectorXf_t msg;
    msg.utime = t*1000*1000;
    msg.rows = 6;

    float vals[6];
    double noise = stopped_noise_;
    if (fabs(last_xyz_dot_.x()) > 0.01 || fabs(last_xyz_dot_.y()) > 0.01 || fabs(last_xyz_dot_.z()) > 0.01)
      noise = moving_noise_;

    vals[0] = x_sign*xyz_ddot_est_.x() + randRange(-noise, noise) + bias_[0];
    vals[1] = x_sign*xyz_ddot_est_.y() + randRange(-noise, noise) + bias_[1];
    vals[2] = xyz_ddot_est_.z() + randRange(-noise, noise) + bias_[2];
    vals[3] = last_rpy_dot_.x() + randRange(-noise, noise) + bias_[3];
    vals[4] = x_sign*last_rpy_dot_.y() + randRange(-noise, noise) + bias_[4];
    vals[5] = x_sign*last_rpy_dot_.z() + randRange(-noise, noise) + bias_[5];
    msg.data = vals;

    mithl_vectorXf_t_publish((lcm_t *) lcm_, channel.c_str(), &msg);
  }
}