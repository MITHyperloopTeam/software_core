#include "PodSim.h"

#include "common_utils.h"

#include <iostream>

#include "lcmgen_c/mithl_floating_base_t.h"

#include <random>
#include <algorithm>

PodSim::PodSim(YAML::Node config, lcm_t * lcm, double t_0)
  {
  lcm_ = lcm;

  //import pod constants
  pod_params_.mass = config["mass"].as<double>();
  pod_params_.crossArea = config["crossSectionalArea"].as<double>();
  pod_params_.dragCoeff = config["dragCoefficient"].as<double>();

  reset(t_0);
}

void PodSim::addComponent(PodComponent *new_component){
  components_.push_back(new_component);
}

void PodSim::reset(double t_0){
  xyz_ = Vec3d(0);
  xyz_dot_ = Vec3d(0);

  rpy_ = Vec3d(0);
  rpy_dot_ = Vec3d(0);

  last_update_time_ = t_0;

  for (auto iter = components_.begin(); iter != components_.end(); iter++){
    (*iter)->reset(t_0);
  }
}

void PodSim::update(double t){
  double dt = t - last_update_time_;
  last_update_time_ = t;


  Vec3d total_force = Vec3d(0);
  Vec3d total_torque = Vec3d(0);

  for (auto iter = components_.begin(); iter != components_.end(); iter++){
    // Component may care about where it is globally. It knows where it is
    // on the pod, so let it do that calculation itself if desired.
    // It'll return a force and torque, both exerted in the origin frame
    // of that component.
    //SimState component_state = (*iter)->update(t, xyz, xyz_dot, rpy, rpy_dot, force, torque);
    Vec3d force, torque;
    (*iter)->update(t, xyz_, xyz_dot_, rpy_, rpy_dot_, force, torque);
    // TODO(gizatt): Do something with SimState

    // force   and torques sum
    force = force.transform(Vec3d(0), (*iter)->rpy_);
    total_force += force;
    total_torque += torque.transform(Vec3d(0), (*iter)->rpy_);
    // but the offset force causes a new torque
    total_torque += (*iter)->xyz_.cross(force);
    //std::cout << "Force: " << force << "and xyz " << (*iter)->xyz_ << " and torque " << (*iter)->xyz_.cross(force) << std::endl;
  }

  // TODO(gizatt): do any damage and health updates
  
  total_force += -1.0 * xyz_dot_*pod_params_.dragCoeff;

  xyz_dot_ += (total_force*dt / pod_params_.mass);
  xyz_ += xyz_dot_*dt;

  // TODO(gizatt): this is the wrong way to integrate rotations and might fail
  rpy_dot_ += total_torque*dt;
  rpy_ += rpy_dot_*dt;
}

void PodSim::output(double t){
  // Let all components output.
  for (auto iter = components_.begin(); iter != components_.end(); iter++){
    (*iter)->output(t);
  }

  //printf("Pod State: t%02.4f, x%02.2f, y%02.2f, z%02.2f, r%02.2f, p%02.2f, y%02.2f\n", t,
  //       xyz_.x(), xyz_.y(), xyz_.z(),
  //       rpy_.x(), rpy_.y(), rpy_.z());

  //printf("X: %02.4f, XD: %02.4f\n", xyz_.x(), xyz_dot_.x());
  // And output ourselves
  
  mithl_floating_base_t msg;

  msg.utime = t*1000*1000;

  msg.q[0] = xyz_.x();
  msg.q[1] = xyz_.y();
  msg.q[2] = xyz_.z();
  msg.q[3] = rpy_.x();
  msg.q[4] = rpy_.y();
  msg.q[5] = rpy_.z();

  msg.v[0] = xyz_dot_.x();
  msg.v[1] = xyz_dot_.y();
  msg.v[2] = xyz_dot_.z();
  msg.v[3] = rpy_dot_.x();
  msg.v[4] = rpy_dot_.y();
  msg.v[5] = rpy_dot_.z();

  mithl_floating_base_t_publish((lcm_t *) lcm_, "SIM_FB", &msg);
}
