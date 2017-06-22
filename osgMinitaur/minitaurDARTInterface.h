// Joe Norby

#ifndef minitaurDARTInterface_H
#define minitaurDARTInterface_H

#include "Motor.h"
//#include "libraries/SPI/SPI.h"
//#include "VectorNav/VN100.h"
//#include "BulkSerial/BulkSerial.h"
#include "libraries/Robot/Behavior.h"
#include "HAL.h"
#include "Remote.h"
#include "SoftStart.h"
//#include "IMUObject.h"

class Interface{

public:
  Interface(void);
  // static const int NUM_BEHAVIORS = 4;
  	
  // Peripheral *remote = &remoteRC;// remoteRC / remoteComputer
  // const bool REMOTE_RC_6CH = true;//false if only 4 channels connected
  // // Peripheral *imu = &imuVN100;// imuVN100 / imuMPU6000
  // // ====== To save compile time if not using MPU6000, comment next two lines =====
  // // #include <MPU6000.h>
  // // #include <Eigen.h>

  // // This must be set per robot zeros must be checked before running!
  // const float motZeros[8] = {0,0,0,0,0,0,0,0}; //DART Mini
  // //const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 1.169, 1.078, 6.252}; //RML Mini
  // //const float motZeros[8] = {2.041, 1.616, 5.522, 2.484, 1.712, 5.356, 0.652, 2.017}; // MLab Mini

  // // Behavior array: add behaviors here. First one in the array is the starting behavior.
  // // Make sure the #include is in Remote.h
  // Behavior *behaviorArray[NUM_BEHAVIORS] = {&bound, &walk, &dig, &pushwalk};

  // volatile uint32_t controlTime = 0;
  // SoftStart softStart;

  // // ======================================================================

  void setup();

  void update();	
  	
private:
};

extern Interface interface;

#endif