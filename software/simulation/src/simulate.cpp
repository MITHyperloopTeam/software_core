#include "PodSim.h"
#include "StateUtils.h"
#include "Vec3d.h"
#include "Pod.h"
#include "Tube.h"
#include "common_utils.h"

// component types
#include "PodComponent_ConstantForcer.h"
#include "PodComponent_BrakeSystem.h"
#include "PodComponent_Pusher.h"
#include "PodComponent_IMU.h"
#include "PodComponent_FiducialSensor.h"
#include "PodComponent_LevForcer.h"

#include <iostream>
#include <iomanip>

#include <lcm/lcm.h>
#include "lcmgen_c/mithl_vectorXf_t.h"
#include "lcmgen_c/mithl_trigger_t.h"
#include "lcmgen_c/mithl_pin_sim_t.h"
#include "lcmgen_c/mithl_velocity_t.h"
#include <unistd.h>

#include <random> 

using namespace std;

string configFile = "../config/simConfig.yaml";

lcm_t * lcm;
PodSim * pod_sim;
double t;

void *lcmMonitor(void *plcm);

static void resetTriggerHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){
  ((PodSim*)user)->reset(getUnixTime());
}

int main(int argc, char** argv)
{

  if(argv[1] != NULL){
    configFile = argv[1];
  }

  YAML::Node config = YAML::LoadFile(configFile);
  auto pod_config = config["pod"];

  t = getUnixTime();

  lcm = lcm_create("udpm://239.255.76.67:62237?ttl=0");

  pod_sim = new PodSim(pod_config, lcm, t);

  // add all components
  if (pod_config["components"]){
    for (auto iter = pod_config["components"].begin(); iter != pod_config["components"].end(); iter++){
      std::string component_type = (*iter)["type"].as<string>();
      if (component_type == "ConstantForcer"){
        ConstantForcer * component = new ConstantForcer(*iter, t);
        pod_sim->addComponent(component);
      } else if (component_type == "BrakeSystem"){
        BrakeSystem * component = new BrakeSystem(*iter, lcm, t);
        pod_sim->addComponent(component);
      } else if (component_type == "Pusher"){
        Pusher * component = new Pusher(*iter, lcm, t);
        pod_sim->addComponent(component);
      } else if (component_type == "IMU"){
        IMU * component = new IMU(*iter, lcm, t);
        pod_sim->addComponent(component);
      } else if (component_type == "FiducialSensor"){
        FiducialSensor * component = new FiducialSensor(*iter, lcm, t, pod_config["tube"]);
        pod_sim->addComponent(component);
      } else if (component_type == "LevForcer"){
        LevForcer * component = new LevForcer(*iter, t);
        pod_sim->addComponent(component);
      } else {
        cout << "Got component type " << component_type << " but I don't know what to do with it!" << endl;
      }
    }
  }

  pthread_t lcmThread;
  pthread_create(&lcmThread, NULL, lcmMonitor, lcm);

  // hook up reset logic
  mithl_trigger_t_subscribe(lcm, "SIM_RESET", &resetTriggerHandler, pod_sim);

  // TODO(gizatt): load all components from YAML
  double dt = 0.001;
  double last_update_time = getUnixTime();
  while (1){
    t = getUnixTime();
    pod_sim->update(t);
    pod_sim->output(t);
    usleep(10);
  }
}

void *lcmMonitor(void *plcm) {
  lcm_t *lcm = (lcm_t *) plcm;
  while (1){
    lcm_handle(lcm);
    usleep(10);
  }
}