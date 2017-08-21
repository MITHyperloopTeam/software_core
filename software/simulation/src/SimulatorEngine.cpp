#include "SimulatorEngine.h"

#include "common_utils.h"

#include <iostream>

#include <random>
#include <algorithm>
#include "yaml-cpp/yaml.h"

SimulatorEngine::SimulatorEngine() : 
  pushingEnabled(0), configured(0)
  {}

SimulatorEngine::SimulatorEngine(std::string &configFile) :
  pushingEnabled(0), configured(0)
  {
  configure(configFile);
}

void SimulatorEngine::configure(std::string &configFile) {  
  pushingEnabled = 0;

  configurationFile = configFile;

  YAML::Node config = YAML::LoadFile(configFile);

  // import simulator parameters
  timestep = config["timestep"].as<double>();
  maxSimulationTime = config["maxSimulationTime"].as<double>();
  maxEndVelocity = config["maxEndVelocity"].as<double>();
  pushingEnabled = config["autoStartSim"].as<int>();
  
  // import tube constants
  tube.length = config["tube"]["length"].as<double>();
  tube.ambientTemp = config["tube"]["ambientTemp"].as<double>();
  tube.pressure = config["tube"]["pressure"].as<double>();

  pushingAcceleration = config["tube"]["pushingAcceleration"].as<double>();
  pushingEnd = config["tube"]["pushingEnd"].as<double>();

  tube.distanceToFirstFiducial = config["tube"]["distanceToFirstFiducial"].as<double>();
  tube.fiducialSeparation = config["tube"]["fiducialSeparation"].as<double>();
  tube.fiducialWidth = config["tube"]["fiducialWidth"].as<double>();
  tube.distanceSensorToFiducial = config["tube"]["distanceSensorToFiducial"].as<double>();
  
  //import pod constants
  pod.mass = config["pod"]["mass"].as<double>();
  pod.crossArea = config["pod"]["crossSectionalArea"].as<double>();
  pod.dragCoeff = config["pod"]["dragCoefficient"].as<double>();
  pod.brakeMu = config["pod"]["brakeMu"].as<double>();

  airSpecificGasConst = 287.058; // (m^3*Pa)/(kg*K)

  tubeAirDensity = calcTubeAirDensity();

  // set initial time to 0
  lastTime = 0.0;
  curTime = timestep;

  // set initial state
  position = Vec3d(0);
  velocity = Vec3d(0);
  rpy = Vec3d(0);
  angVelocity = Vec3d(0);
  acceleration = Vec3d(0);

  brakeForce = 0;
  lowSpeedDesiredVelocity = 0.0;
  lowSpeedEnabled = 0;

  // set initial sensor values
  fiducialReading = 0.0;
  imuReading = 0.0;

  configured = 1;

  std::cout << "Engine: simulator configured: " << configFile << std::endl;
}

double SimulatorEngine::calcTubeAirDensity() {
  return tube.pressure / ( airSpecificGasConst * tube.ambientTemp );
}

int SimulatorEngine::updateState() {

  double pushing = samplePushingAcceleration();

  Vec3d forcedAcceleration = Vec3d(pushing, 0, 0);
  Vec3d dragAcceleration = calcDragForce() / pod.mass;
  Vec3d brakeAcceleration = Vec3d( - sgnd(velocity.x()) * ( brakeForce * pod.brakeMu ) / pod.mass, 0, 0);
  Vec3d lowSpeedAccleration = calcLowSpeedAcceleration();
  Vec3d totalAcceleration = forcedAcceleration + dragAcceleration + brakeAcceleration 
    + lowSpeedAccleration;

  // add noise to acceleration
  // TODO change randomness
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> r(0.0, 0.01);
  Vec3d noise(r(gen), r(gen), r(gen));
  totalAcceleration = totalAcceleration + noise;

  Vec3d velChange = totalAcceleration * timestep;
  Vec3d displacement = velocity * timestep;

  position = position + displacement;
  velocity = velocity + velChange;
  acceleration = totalAcceleration;
}

int SimulatorEngine::inCollision() {
  if(position.x() > tube.length || position.x() < -0.01)
    return 1;
  else
    return 0;
}

int SimulatorEngine::updateSensors() {
  fiducialReading = sampleFiducialSensor();
  imuReading = sampleIMU();
  return 1;
}

int SimulatorEngine::sampleFiducialSensor() {

  // TODO add some uncertainty.  Maybe give some false 
  // positives/negatives some % of samplings?

  double tempDistance = position.x() - tube.distanceToFirstFiducial;
  if(tempDistance > 0 && tempDistance < tube.fiducialWidth)
    return 1;

  while(tempDistance > 0) {
    if(tempDistance < tube.fiducialWidth) {
      return 1;
    }
    tempDistance = tempDistance - tube.fiducialSeparation;
  }
  return 0;
}

double SimulatorEngine::sampleIMU() {

  // TODO change uncertainty distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> r(0.0, 0.001);

  return acceleration.x() + r(gen);
}

double SimulatorEngine::samplePushingAcceleration() {
  if(!pushingEnabled)
    return 0.0;
  if (position.x() < pushingEnd)
    return pushingAcceleration;
  else {
    pushingEnabled = 0;
    return 0.0;
  }
}

Vec3d SimulatorEngine::calcDragForce() {
  double force = 0.5*tubeAirDensity*velocity.x()*velocity.x()*pod.dragCoeff*pod.crossArea;
  return Vec3d(-force, 0, 0);
}

Vec3d SimulatorEngine::calcLowSpeedAcceleration() {
  if(!lowSpeedEnabled)
    return Vec3d(0,0,0);
  if(samplePushingAcceleration() != 0){
    std::cout << "Engine: WARNING, low speed motors ON during forced acceleration" << std::endl;
  }
  if(velocity.x() > 2){
    std::cout << "Engine: WARNING, low speed motors ON when velocity is > 1 m/s" << std::endl;
  }

  double err = lowSpeedDesiredVelocity - velocity.x(); 
  double a = err * 1;

  if (a > 0)
    a = std::max(std::min(a, 1.0), 0.002);
  else
    a = std::min(std::max(a, -1.0), -0.002);

  return Vec3d(a, 0, 0);
}


int SimulatorEngine::run(void (*callback)()) {
  
  if(!configured) {
    std::cout << "Engine: Simulator has not been configured. Need to run configure().  Aborting simulation." << std::cout;
    return 0;
  }
  std::cout << "Engine: Simulator ready to run..." << std::endl;

  bool collision_msg_printed = false;
  while(1) {

    // check if we collided with anything
    if(inCollision()) {
      if (!collision_msg_printed)
        std::cout << "Engine: There was a crash!! :(" << std::endl;
      collision_msg_printed = true;
      sleep(1.0);

    } else {
      // update simulator
      collision_msg_printed = false;
      updateState();
      updateSensors();
      // step forward in time
      lastTime = curTime;
      curTime = curTime + timestep;
    }

    callback();


  }
  return 1;
}