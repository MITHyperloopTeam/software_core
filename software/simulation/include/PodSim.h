#ifndef POD_SIM_H
#define POD_SIM_H

#include "Pod.h"
#include "PodComponent.h"
#include "Vec3d.h"
#include <vector>
#include "yaml-cpp/yaml.h"
#include <lcm/lcm.h>

class PodSim {
  /**
   *  Manages the collection of pod components.
   */

  public:
    PodSim(YAML::Node config, lcm_t * lcm, double t_0);
    void reset(double t_0);
    void addComponent(PodComponent * new_component);
    void update(double t);
    void output(double t);

  private:
    // how pod is built:
    Pod pod_params_;
    std::vector<PodComponent *> components_;
    
    double last_update_time_;

    // pod floating base state:
    Vec3d xyz_;
    Vec3d rpy_;
    Vec3d xyz_dot_;
    Vec3d rpy_dot_;

    lcm_t * lcm_;
};

#endif // POD_SIM_H