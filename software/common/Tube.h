#ifndef TUBE_H
#define TUBE_H

struct Tube {
  double length;
  double ambientTemp;
  double pressure;
  
  double distanceToFirstFiducial;
  double fiducialSeparation;
  double fiducialWidth;
  double distanceSensorToFiducial; 
};

#endif // TUBE_H