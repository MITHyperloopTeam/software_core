#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "Vec3d.h"
#include "Tube.h"
#include "Pod.h"
#include "ParticleFilter.h"

#include <zcm/zcm.h>

#include "zcmgen_c/mithl_state_t.h"

class StateEstimator {
  zcm_t * zcm;

  double position;
  double velocity;

  double lastTime; // seconds
  double curTime; //seconds

  double imuReading;
  double imuNoise = 0.1;
  double dynamicsNoise = 0.1;
  double damping_mu = 0.1;
  //double accNoise = 0.00001;
  //double velNoise = 0.00001;
  double fiducialReading;

  double lastBrakingCommand;

  struct Pod pod;
  struct Tube tube;

  double lastPublishTime = 0.0;

  int arm = MITHL_STATE_T_ARM;
  int launch = MITHL_STATE_T_LAUNCH;
  int flight = MITHL_STATE_T_FLIGHT;
  int safe = MITHL_STATE_T_SAFE;
  int floating = MITHL_STATE_T_FLOATING;
  int drive = MITHL_STATE_T_DRIVE;
  int soft_stop = MITHL_STATE_T_SOFT_STOP;
  int estop = MITHL_STATE_T_ESTOP;
  int fault = MITHL_STATE_T_FAULT;

  int currentState = safe;
  
  int noUpdateStates[6]  = {arm, floating,soft_stop,estop, fault, safe}; 
  int fiducialUpdateStates[2] = {launch,flight}; 
  int regularUpdateStates[1]  = {drive}; 

  DualParticleFilter pf;
  
   public:
    StateEstimator(struct Tube tube, struct Pod pod, zcm_t * zcm);
    ~StateEstimator();

    void reset();
    void publish();

    //State getState() {std::cout << "se state" << state.pos.x << std::endl;return state;};
    int modelUpdate();
    int sensorUpdate();

    void setCurTime(double t) {lastTime = curTime; curTime = t; };
    void setIMU(double r) { imuReading = r; };
    void setFiducial(double r) {fiducialReading = r; };
    void setBrakingCommand(double r) {lastBrakingCommand = r; };

    double getPosition() { return position; };
    double getVelocity() { return velocity; };
    void setPosition(double position_) { this->position = position_; };
    void setVelocity(double velocity_) { this->velocity = velocity_; };
    void setState(int currentState_) { this->currentState = currentState_; };
    bool inarray (int number, int array[], int array_length) {
      for(int i=0;i<=array_length;i++){
        if(array[i]==number){
          return true;
        }
      }
      return false;
    }

    bool useDualSampling(double dt, float * particle);
    void processSample(double dt, float * particle);
    float processProba(double dt, float * particle);
    void measurementSample(double dt, float * particle);
    float measurementProba(double dt, float * particle);

};

#endif // STATE_ESTIMATOR_H
