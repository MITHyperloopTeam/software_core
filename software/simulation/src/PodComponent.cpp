#include "PodComponent.h"

#include "common_utils.h"

#include <iostream>

#include <random>
#include <algorithm>

PodComponent::PodComponent(YAML::Node config, double t_0)
{
  if (config["xyz"])
  	xyz_ = Vec3d(config["xyz"][0].as<double>(),
  		          config["xyz"][1].as<double>(),
  		          config["xyz"][2].as<double>());
  else
  	xyz_ = Vec3d(0.0);

  if (config["rpy"])
  	rpy_ = Vec3d(config["rpy"][0].as<double>(),
  		          config["rpy"][1].as<double>(),
  		          config["rpy"][2].as<double>());
  else
  	rpy_ = Vec3d(0.0);  

  reset(t_0);
}

void PodComponent::reset(double t_0){
  last_update_time_ = t_0;
}