#include <lcm/lcm.h>
#include <yaml-cpp/yaml.h>
#include "lcmgen_c/mithl_config_t.h"
#include "lcmgen_c/mithl_trigger_t.h"
#include "lcmgen_c/mithl_pod_t.h"
#include "lcmgen_c/mithl_tube_t.h"

using namespace std;

lcm_t *lcm;
mithl_config_t * config;

void loadParams(std::string &file) {
  YAML::Node rawConfig = YAML::LoadFile(file);
  
  config->utime = 0;
  // import tube constants
  config->tube.length = rawConfig["tube"]["length"].as<float>();
  config->tube.ambientTemp = rawConfig["tube"]["ambientTemp"].as<float>();
  config->tube.pressure = rawConfig["tube"]["pressure"].as<float>();

  config->tube.distanceToFirstFiducial = rawConfig["tube"]["distanceToFirstFiducial"].as<float>();
  config->tube.fiducialSeparation = rawConfig["tube"]["fiducialSeparation"].as<float>();
  config->tube.fiducialWidth = rawConfig["tube"]["fiducialWidth"].as<float>();
  config->tube.distanceSensorToFiducial = rawConfig["tube"]["distanceSensorToFiducial"].as<float>();
  
  //import pod constants
  config->pod.mass = rawConfig["pod"]["mass"].as<float>();
  config->pod.crossSectionalArea = rawConfig["pod"]["crossSectionalArea"].as<float>();
  config->pod.dragCoefficient = rawConfig["pod"]["dragCoefficient"].as<float>();
  config->pod.brakeMu = rawConfig["pod"]["brakeMu"].as<float>();
  
}

static void request_handler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){

  // set sending time
  // todo: give a real time
  config->utime = 0;

  mithl_config_t_publish(lcm, "CNFG", config);
}


int main(int argc, char** argv) {

  string file = "../../config/simConfig.yaml";
  if(argv[1] != NULL){
    file = argv[1];
  }


  config = new mithl_config_t;


  loadParams(file);

  lcm = lcm_create("udpm://239.255.76.67:62237?ttl=0");

  mithl_trigger_t_subscribe(lcm, "CNFG_RQST", &request_handler, NULL);

  while(1)
    lcm_handle(lcm);
}