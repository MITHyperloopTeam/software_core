#include "PodComponent_ConstantForcer.h"

#include "common_utils.h"

#include <iostream>

#include <random>
#include <algorithm>

ConstantForcer::ConstantForcer(YAML::Node config, double t_0)
                              : PodComponent(config, t_0)
{
  if (config["force"])
  	force_ = Vec3d(config["force"][0].as<double>(),
  		          config["force"][1].as<double>(),
  		          config["force"][2].as<double>());
  else
  	force_ = Vec3d(0.0);

  if (config["torque"])
  	torque_ = Vec3d(config["torque"][0].as<double>(),
  		          config["torque"][1].as<double>(),
  		          config["torque"][2].as<double>());
  else
  	torque_ = Vec3d(0.0);  
}

void ConstantForcer::update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque)
{
  force = force_;
  torque = torque_;
}

void ConstantForcer::output(double t)
{
  //printf("Constant Force output.\n");
}
