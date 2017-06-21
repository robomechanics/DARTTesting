/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Remote_h
#define Remote_h

//#include "arduino/avr/cores/arduino/Arduino.h"
#include "libraries/Robot/Behavior.h"
#include "Peripheral.h"

// ===== Include behaviors here =====
#include "Bound.h"
#include "Walk.h"
#include "Dig.h"
#include "pushWalk.h"
// ==================================

extern Behavior *behavior;
extern float rcCmd[6];

// behavior array: add behaviors here
extern const int NUM_BEHAVIORS;
extern Behavior *behaviorArray[];

// function to programmatically change behavior
void activateBehavior(Behavior *behav);

// parameters for various remotes
#define REMOTE_RC_ZERO 7.87
#define REMOTE_SIGNAL_HYSTERESIS 2000

extern Peripheral *remote;// defined in .ino file

// RC remote class
class RemoteRC : public Peripheral {
public:
  // local
  DLPF speedDesF, yawDesF;
// remote
  volatile bool running, throttle;
  volatile uint32_t throttleChangeCounter;// for debouncing
  const static int NRECPINS = 6;
  uint32_t lastSignal = 0;
  // RC receiver pins
  //const uint8_t rcRecPin[6] = {PC15, PC14, PC13, PB7, PB6, PD3};
  // const uint8_t rcRecPin[] = {PB7, PB6, PD3};
  // volatile float rcCmd[NRECPINS];
  bool use6Channels = false;

  // called from setup()
  virtual void begin();
  // called from controlLoop(), don't use delay()
  virtual void updateInterrupt();
  // called from loop(): can use delay()
  virtual void updateLoop();
};
extern RemoteRC remoteRC;
extern const bool REMOTE_RC_6CH;

// Use computer as remote
//#include "BulkSerial/BulkSerial.h"

// TODO need to timeout and go to stand
/*
struct ComputerPacket {
  uint16_t align;
  uint8_t cmd;
  // float param1;
  // float param2;
  float params[16];
  uint16_t checksum;
} __attribute__ ((packed));

class RemoteComputer : public Peripheral {
public:
  const static int TIMEOUT = 500;
  const static uint8_t CMD_KILL   = 0;
  const static uint8_t CMD_STAND  = 1;
  const static uint8_t CMD_START  = 2;
  const static uint8_t CMD_SET_POSITION = 3;
  const static uint8_t CMD_SET_OPEN_LOOP = 4;
  const static uint8_t CMD_SET_GAIT_BOUND = 5;
  const static uint8_t CMD_SET_GAIT_WALK = 6;
  //other gaits...
  const static uint8_t CMD_SIGNAL0 = 10;
  const static uint8_t CMD_SIGNAL1 = 11;
  const static uint8_t CMD_SIGNAL2 = 12;
  const static uint8_t CMD_SIGNAL3 = 13;

  // local
  uint32_t lastSignal = 0;
  DLPF speedDesF, yawDesF;

  RemoteComputer() : rpi(MBLC_RPI) {}
  // Raspberry pi
  BulkSerial rpi;
  ComputerPacket computerPacket = {0,0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},0};
  uint32_t lastRx = 0, lastTx = 0;
  volatile uint32_t lastCompPacket = 0;
  bool firstCompPacket = false;

  // called from setup()
  virtual void begin();
  // called from controlLoop(), don't use delay()
  virtual void updateInterrupt();
  // called from loop(): can use delay()
  virtual void updateLoop();
};
extern RemoteComputer remoteComputer;

// Nunchuck as remote

// #include "Nunchuck.h"
// class RemoteNunchuck : public Peripheral {
// need debounce for Z
// uint32_t lastZPress = 0, lastCPress = 0;
//   Nunchuck nunchuck;

  // // called from setup()
  // virtual void begin();
  // // called from controlLoop(), don't use delay()
  // virtual void updateInterrupt(Behavior *behavior);
  // // called from loop(): can use delay()
  // virtual void updateLoop(Behavior *behavior);
// };
// extern RemoteNunchuck remoteNunchuck;

// =================================================
*/
#endif
