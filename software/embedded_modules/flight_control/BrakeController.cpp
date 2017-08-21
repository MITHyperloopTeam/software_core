#include "BrakeController.h"
#include <stdio.h>

BrakeController::BrakeController(struct Tube tube, struct Pod pod) :
  pod(pod), tube(tube) {
    reset();

    brakingSlope = -2.0;
    desiredDistanceToEnd = 50;
    kP = 100.0;

}

//not finding the subsciption to the braking slope message type,
void BrakeController::reset() {
  // init state
  maxBrakingGs = 2.2;
  maxBrakeForce = (maxBrakingGs * 9.81 * pod.mass) / pod.brakeMu;
  
  commandedForce = 0;

}

void BrakeController::setBrakingSlope(double brakingSlope_){
  brakingSlope = brakingSlope_;
}

void BrakeController::setDistanceToEnd(double desiredDistanceToEnd_){
  desiredDistanceToEnd = desiredDistanceToEnd_;
};  

void BrakeController::setProportionalConstant(double kP_){
  kP = kP_;
}

// Currently assumes that we can apply a constant normal force to the rail and that 
// constant force has a constant braking effect 
double BrakeController::output(double state[]) { 
  if (this->currentState == 5) {
    return 0; // don't break from teleop mode
  } //teleop
  distanceToStop = (tube.length - desiredDistanceToEnd) - state[0]; 
  desiredVelocity = distanceToStop*(-brakingSlope); //r = desiredVelocity <-- currently linear ramp downwards
  currentVelocity = state[6];
  
  if (currentVelocity < desiredVelocity){
    desiredVelocity = currentVelocity;
  }

  if(currentState == floating || currentState == launch || currentState == arm){
    return 0;
  }
  if (currentState == soft_stop || currentState == estop || currentState == safe || currentState == fault){
    return maxBrakeForce;
  } 
  //should be in flight mode
  
  commandedForce = kP*(currentVelocity-desiredVelocity);

  if (commandedForce > maxBrakeForce) { 
    return maxBrakeForce;
  } 

  return commandedForce;
}

