#ifndef SIMULATOR_ENGINE_H
#define SIMULATOR_ENGINE_H

#include "StateUtils.h"
#include "Vec3d.h"
#include "Pod.h"
#include "Tube.h"

class SimulatorEngine {
  
  // The simulated true state of the pod //
  Vec3d position;
  Vec3d velocity;
  Vec3d rpy;
  Vec3d angVelocity;

  Vec3d acceleration;


  double lastTime;
  double curTime;


  std::string configurationFile;
  //****  Constants to be loaded from yaml file  *****//
  // simulator constants
  double timestep;  // seconds
  double maxSimulationTime;
  double maxEndVelocity;

  double pushingAcceleration;
  double pushingEnd;
  int pushingEnabled;

  //***********   End constants          *************//

  
  struct Pod pod;
  struct Tube tube;

  double tubeAirDensity;  // calculated on sim configuration
  double airSpecificGasConst;  // set in configure()
  double calcTubeAirDensity();


  // Sensor readings
  double fiducialReading;
  double imuReading;

  // generates all sensor outputs
  int updateSensors();
  int sampleFiducialSensor();
  double sampleIMU();

  double samplePushingAcceleration();
  Vec3d calcDragForce();

  double brakeForce;

  // low speed
  int lowSpeedEnabled;
  double lowSpeedDesiredVelocity;
  Vec3d calcLowSpeedAcceleration();

  // check to see if pod is in collision //
  int inCollision();

  int configured;

  // Dynamics model update using given timestep //
  int updateState();


  static void lcmMonitor(void *s);

  public:
    SimulatorEngine();
    SimulatorEngine(std::string &);
    
    void configure(std::string &);

    // UI signal flags    
    double startRun() { pushingEnabled = 1; };

    // Getters
    Vec3d getPosition() { return position; };
    Vec3d getVelocity() { return velocity; };
    Vec3d getRPY() { return rpy; };
    Vec3d getAngVelocity() { return angVelocity; };

    double getCurTime() { return curTime; };
    double getFiducialReading() { return fiducialReading; };
    double getImuReading() { return imuReading; };

    struct Tube getTube() { return tube; };
    struct Pod getPod() { return pod; };

    void setBrakeForce(double newForce) {brakeForce = newForce; };

    void setLowSpeedVelocity(double v) { lowSpeedDesiredVelocity = v; };
    void lowSpeedEnable() { lowSpeedEnabled = 1; };
    void lowSpeedDisable() { lowSpeedEnabled = 0; };

    int run(void (*callback)());

};

#endif // SIMULATOR_ENGINE_H