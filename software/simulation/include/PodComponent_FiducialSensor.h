#ifndef FiducialSensor_H
#define FiducialSensor_H

#include "PodComponent.h"
#include "Vec3d.h"
#include "yaml-cpp/yaml.h"
#include <lcm/lcm.h>
#include "string.h"

class FiducialSensor : public PodComponent {
  public:
    FiducialSensor(YAML::Node config, lcm_t * lcm, double t_0, YAML::Node tube_config);
    void reset(double t_0);
    void update(double t, const Vec3d xyz, const Vec3d xyz_dot, const Vec3d rpy, const Vec3d rpy_dot, Vec3d & force, Vec3d & torque);
    void output(double t);
  private:
    lcm_t * lcm_;
    std::string channel_;
    
    double last_broadcast_time_;
    double last_update_time_;
    int total_count_;
    double last_distance_since_last_strip_;
    double last_saw_strip_;
    double last_strip_duration_;

   	double broadcast_period_ = 0.01;
    // All units meters unless otherwise specified
    double fiducial_separation_ = 30.48;
    double distance_after_last_fiducial_ = 30.48;
    double fiducial_width_ = 0.1016; 
    double tube_length_ = 1200;

    double false_positive_rate_per_meter_ = 0.0;
    double true_positive_rate_per_strip_ = 1.0;
    double strip_duration_jitter_ = 0.0;
    double strip_width_bias_ = 0.0;
};

#endif