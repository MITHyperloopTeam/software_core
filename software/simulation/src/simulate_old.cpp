#include "SimulatorEngine.h"
#include "StateUtils.h"
#include "Vec3d.h"
#include "Pod.h"
#include "Tube.h"

#include <iostream>
#include <iomanip>

#include <lcm/lcm.h>
#include "lcmgen_c/mithl_vectorXf_t.h"
#include "lcmgen_c/mithl_trigger_t.h"
#include "lcmgen_c/mithl_pin_sim_t.h"
#include "lcmgen_c/mithl_velocity_t.h"
#include <unistd.h>

#include <random> 

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

using namespace std;

string configFile = "../config/simConfig.yaml";

lcm_t * lcm;

SimulatorEngine * simEngine;

int simCount = 0;

double startTime;

void *lcmMonitor(void *f);
static void startHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user);
static void resetHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user);
static void lowSpeedHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_velocity_t * msg, void * user);
static void lowSpeedEnableHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){
  simEngine->lowSpeedEnable();
}
static void lowSpeedDisableHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){
  simEngine->lowSpeedDisable();
}
static void brake_input_handler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_vectorXf_t * msg, void * user);


double lastPublishedGroundTruth = 0.0;
double lastPublishedAnalogValues = 0.0;
double lastPublishedTimeTime = 0.0;
double lastPublishedIMU = 0.0;

double analog_setpoint[11];

void callbackFunc(){
  if (simEngine->getCurTime() - lastPublishedGroundTruth > 0.01){
    cout << "Time: "  << setprecision(5) << simEngine->getCurTime() 
    << " s \t\tSimPos: " << setprecision(5) << simEngine->getPosition().x() 
     <<  "\tSimVel: " << setprecision(5) << simEngine->getVelocity().x() << endl;
  
    mithl_vectorXf_t msg;
    int rows = 2;
    float buf[rows];
    buf[0] = simEngine->getPosition().x();
    buf[1] = simEngine->getVelocity().x();
    msg.utime = (int) (simEngine->getCurTime() * 1000000.);
    msg.rows = rows;
    msg.data = buf;

    mithl_vectorXf_t_publish(lcm, "SIM_FB", &msg);
    lastPublishedGroundTruth = simEngine->getCurTime();
  }

  if (simEngine->getCurTime() - lastPublishedIMU > 0.01){
    mithl_vectorXf_t msg;
    msg.utime = (int) (simEngine->getCurTime() * 1000000.);
    msg.rows = 6;
    float buf[6];
    for (int i=0; i < 6; i++) buf[i] = 0.0;
    buf[0] = simEngine->getImuReading();
    msg.data = buf;
    mithl_vectorXf_t_publish(lcm, "SIM_IMU", &msg);
    lastPublishedIMU = simEngine->getCurTime();
  }

  if (simEngine->getCurTime() - lastPublishedAnalogValues > 0.05){
    mithl_pin_sim_t msg;
    msg.utime = (int) (simEngine->getCurTime() * 1000000.);
    for (int i=0; i<sizeof(msg.analog)/sizeof(msg.analog[0]); i++){
      analog_setpoint[i] += i*(((float)random()) / RAND_MAX - 0.5);
      msg.analog[i] = analog_setpoint[i] + (((float)random()) / RAND_MAX - 0.5)*1;
    }
    for (int i=0; i<sizeof(msg.digital)/sizeof(msg.digital[0]); i++)
      msg.digital[i] = -1;
    mithl_pin_sim_t_publish(lcm, "SIM_PINS", &msg);
    lastPublishedAnalogValues = simEngine->getCurTime();
  }

  if (simEngine->getCurTime() - lastPublishedTimeTime > 0.5){
    mithl_trigger_t msg;
    msg.utime = (int) (simEngine->getCurTime() * 1000000.);
    mithl_trigger_t_publish(lcm, "TIME", &msg);
    lastPublishedTimeTime = simEngine->getCurTime();
  }

  double elapsed_actually = getUnixTime() - startTime;
  double elapsed_sim = simEngine->getCurTime();
  if (elapsed_sim > elapsed_actually){
    usleep(1000*1000*(elapsed_sim - elapsed_actually));
  }
  simCount++;
}

int main(int argc, char** argv)
{

  if(argv[1] != NULL){
    configFile = argv[1];
  }

  for (int i=0; i < 11; i++){
    analog_setpoint[i] = 1000 * ((float)random()) / RAND_MAX;
  }
  startTime = getUnixTime();

  lcm = lcm_create("udpm://239.255.76.67:62237?ttl=0");

  simEngine = new SimulatorEngine();
  simEngine->configure(configFile);


  mithl_trigger_t_subscribe(lcm, "START", &startHandler, NULL);
  mithl_trigger_t_subscribe(lcm, "RESET", &resetHandler, NULL);
  mithl_vectorXf_t_subscribe(lcm, "_LSM_B", &brake_input_handler, NULL);
  mithl_velocity_t_subscribe(lcm, "TELEOP_VELOCITY", &lowSpeedHandler, NULL);
  mithl_trigger_t_subscribe(lcm, "TELEOP_ENABLE", &lowSpeedEnableHandler, NULL);
  mithl_trigger_t_subscribe(lcm, "TELEOP_DISABLE", &lowSpeedDisableHandler, NULL);


  pthread_t lcmThread;
  pthread_create(&lcmThread, NULL, lcmMonitor, lcm);

  simEngine->run(callbackFunc);
}

static void brake_input_handler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_vectorXf_t * msg, void * user){
  simEngine->setBrakeForce(msg->data[0]);
}

static void startHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){
  simEngine->startRun();
  cout << "STARTING" << endl;
}

static void resetHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){
  cout << "RESETTING" << endl;
  simEngine->configure(configFile);
  lastPublishedGroundTruth = 0;
  lastPublishedAnalogValues = 0;
  lastPublishedTimeTime = 0;
  lastPublishedIMU = 0;
  startTime = getUnixTime();
}

static void lowSpeedHandler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_velocity_t * msg, void * user){
  simEngine->setLowSpeedVelocity(msg->desiredVelocity);
}

void *lcmMonitor(void *plcm) {
  lcm_t *lcm = (lcm_t *) plcm;
  while (1)
    lcm_handle(lcm);
}