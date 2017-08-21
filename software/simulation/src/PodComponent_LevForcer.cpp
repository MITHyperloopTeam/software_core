#include "PodComponent_LevForcer.h"

#include "common_utils.h"

#include <iostream>

#include <random>
#include <algorithm>

/**
 * Implements the forces for lev model as characterised in:
 *
 *  analysis/State Est Devel.ipynb
 *
 * 
 */

using namespace std;

LevForcer::LevForcer(YAML::Node config, double t_0)
                              : PodComponent(config, t_0)
{
  //pod_mass_ = config["podmass"] // 260 kg
  //podlev_center_ = config["podlev_center"] // 8
  //podlev_slope_ = config["podlev_slope"] // 1.5
  //podlev_height_ = config["podlev_height"] // .0015 
  podlev_drag_peak_speed_ = config["podlev_drag_peak_speed"].as<double>();
  podlev_drag_peak_magnitude_ = config["podlev_drag_peak_magnitude"].as<double>();
  podlev_drag_peak_spread_ = config["podlev_drag_peak_spread"].as<double>();
  podlev_drag_above_peak_ = config["podlev_drag_above_peak"].as<double>();
  podlev_drag_above_peak_slope_ = config["podlev_drag_above_peak_slope"].as<double>();
}

void LevForcer::update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque)
{
  // relevant values are speed, height, (pitching), 
  double height = xyz.z();
  double velocity = xyz_dot.x();

  //  // Height here is in range [-1,1]. only for ratio>0 is pod above ground. pod will experience lev force
  //  // proportional to difference betweeen its current height and the expected lev height.
  //  double lev_height_ratio_from_vel = (2.0 / (1 + exp((podlev_center_ - velocity) * podlev_slope_))) - 1.0;
  //  double pod_gravity_force_mag = pod_mass_ * gravity_;   // Newtons
  //
  //  // Force follows a left-of-0 inverse-square law on height, scaled such that the height-velocity curve is
  //  // as expected.
  //  double lev_vertical_force_mag = 0; // None for now.

  double lev_drag_force_mag = LevForcer::get_drag_force(velocity);

  force = Vec3d(-lev_drag_force_mag, 0, 0);
  torque = Vec3d(0.0);
}

double LevForcer::get_drag_force(double velocity) {
  double lev_drag_force_mag = podlev_drag_above_peak_ * (2 / (1 + exp(-velocity * podlev_drag_above_peak_slope_)) - 1) \
      + podlev_drag_peak_magnitude_ * (exp( -pow(podlev_drag_peak_speed_ - velocity, 2) / podlev_drag_peak_spread_));

  return lev_drag_force_mag;
}

void LevForcer::output(double t)
{
  // pass
}

