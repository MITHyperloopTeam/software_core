#include "PodComponent_FiducialSensor.h"

#include "common_utils.h"

#include <iostream>

#include <random>
#include <algorithm>
#include "lcmgen_c/mithl_fiducial_t.h"

static inline double randRange(double low, double high){
  return ((high - low)*((double)rand() / double(RAND_MAX)) + low);
}

FiducialSensor::FiducialSensor(YAML::Node config, lcm_t * lcm, double t_0, YAML::Node tube_config)
                              : PodComponent(config, t_0), lcm_(lcm)
{
  printf("Initializing an FiducialSensor simulation\n");
  if (config["broadcast_period"])
    broadcast_period_ = config["broadcast_period"].as<double>();
  if (config["channel"])
    channel_ = config["channel"].as<std::string>();
  if (tube_config["fiducial_separation"])
    fiducial_separation_ = config["fiducial_separation"].as<double>();
  if (tube_config["distance_after_last_fiducial"])
    distance_after_last_fiducial_ = config["distance_after_last_fiducial"].as<double>();
  if (tube_config["fiducial_width"])
    fiducial_width_ = config["fiducial_width"].as<double>();
  if (tube_config["fiducial_separation"])
    fiducial_separation_ = config["fiducial_separation"].as<double>();
  if (tube_config["length"])
    tube_length_ = config["tube_length"].as<double>();

  if (config["false_positive_rate_per_meter"])
    false_positive_rate_per_meter_ = config["false_positive_rate_per_meter"].as<double>();
  if (config["true_positive_rate_per_strip"])
    true_positive_rate_per_strip_ = config["true_positive_rate_per_strip"].as<double>();
  if (config["strip_duration_jitter"])
    strip_duration_jitter_ = config["strip_duration_jitter"].as<double>();
  if (config["strip_width_bias"])
    strip_width_bias_ = config["strip_width_bias"].as<double>();

  reset(t_0);
}

void FiducialSensor::reset(double t_0){
  last_update_time_ = t_0;
  last_broadcast_time_ = t_0;
  total_count_ = 0;
  last_saw_strip_ = t_0;
  last_strip_duration_ = 0.0;
  last_distance_since_last_strip_ = fiducial_separation_;
}

void FiducialSensor::update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque)
{
  double dt = t - last_update_time_;
  last_update_time_ = t;

  // Have we passed a strip?
  double distance_to_first_strip = fmod(tube_length_, fiducial_separation_);
  if (xyz.x() >= distance_to_first_strip){
    double distance_since_last_strip = fmod(xyz.x() - distance_to_first_strip, fiducial_separation_);
    if (distance_since_last_strip < last_distance_since_last_strip_){
      // DETECTION!
      double val = randRange(0.0, 1.0);
      if (val <= true_positive_rate_per_strip_){
        last_saw_strip_ = t;
        last_strip_duration_ = (fiducial_width_ + strip_width_bias_) / xyz_dot.x() + randRange(-strip_duration_jitter_, strip_duration_jitter_);
        total_count_++;
      }
    }
    last_distance_since_last_strip_ = distance_since_last_strip;
  }

  double val = randRange(0.0, 1.0);
  if (val < false_positive_rate_per_meter_*dt*xyz_dot.x()){
    last_saw_strip_ = t;
    last_strip_duration_ = (fiducial_width_ + strip_width_bias_) / (randRange(0.0, 2.0)*xyz_dot.x()) + randRange(-strip_duration_jitter_, strip_duration_jitter_);
    total_count_++; 
  }
  
  xyz.x();
  xyz_dot.x();

  force = Vec3d(0.0);
  torque = Vec3d(0.0);
}

void FiducialSensor::output(double t)
{
  if (t - last_broadcast_time_ >= broadcast_period_){
    last_broadcast_time_ = t;
    mithl_fiducial_t msg;
    msg.utime = t*1E6;
    msg.time_since_last = t - last_saw_strip_;
    msg.average_time_strip = last_strip_duration_;
    msg.total_count = total_count_;
    mithl_fiducial_t_publish((lcm_t *) lcm_, channel_.c_str(), &msg);
  }
}