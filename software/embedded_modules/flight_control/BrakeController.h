#ifndef BRAKE_CONTROLLER_H
#define BRAKE_CONTROLLER_H

#include "Tube.h"
#include "Pod.h"

#include "zcmgen_c/mithl_state_t.h"


class BrakeController {

private:
  

 struct Pod pod;
 struct Tube tube;

 double maxBrakeForce;
 double maxBrakingGs;
 
 double desiredDistanceToEnd = 50.0;
 double distanceToStop;

 double kP = 100.0;
 double brakingSlope = -0.17;
 
 double currentVelocity;
 double desiredVelocity;
 double commandedForce;
 
 
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
  int regularUpdateStates[2]  = {arm, drive}; 

public:
  BrakeController(struct Tube tube, struct Pod pod);
  void reset();
  void setBrakingSlope(double brakingSlope_);
  void setDistanceToEnd(double desiredDistanceToEnd_);
  void setProportionalConstant(double kP_);
  double getBrakingForce() {return commandedForce; };
  double output(double state[]);
  void setState(int currentState_) { this->currentState = currentState_; };  
  bool inarray (int number, int array[], int array_length) {
    for(int i=0;i<=array_length;i++){
      if(array[i]==number){
        return true;
      }
    }
    return false;
  }
};

#endif // BRAKE_CONTROLLER_H
