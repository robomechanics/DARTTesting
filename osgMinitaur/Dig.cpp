/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "Dig.h"

Dig dig;

void Dig::begin() {
}

void Dig::update() {
  // std::cout << "Dig update" << std::endl;
  MinitaurLeg::useLengths = false;
  
  float standAng = 0,standExt = 1.57; 
  float kExt = 0.4, kAng = 0.7;
  float tc = 3;
  int tReady = 1000/tc;
  int tLower = 500/tc;
  int tSweep = 2000/tc;
  int tRaise = tLower;
  int tReturn = tReady;
  float sweep = -0.5;
  float lift = -1.2;

  int time = X.t % (tLower+tReady+tSweep+tRaise+tReturn);
  //std::cout << "Motor vel from getVelocity is " << M[5].getVelocity() << std::endl;
  //std::cout << "Motor vel from DART is " << DARTMotorVel[5] << std::endl;

  for(int i=0; i<4; ++i){
    // M[i].setOpenLoop(1);
    // M[i].setGain(10);
    // M[i].setPosition(1.5*PI);
    leg[i].setGain(ANGLE,1);
    leg[i].setGain(EXTENSION,1);
    leg[i].setPosition(ANGLE,0.25*PI);
    leg[i].setPosition(EXTENSION,0*PI);
    // std::cout << leg[i].getPosition(ANGLE) << '\t';
    // std::cout << leg[i].getPosition(EXTENSION) << '\t';
  }
  // std::cout << std::endl;

  for(int i=0;i<4;++i){
    //std::cout << leg[i].getPosition(ANGLE) << '\t';
  }


/*
  for(int i=0; i<4; ++i){
    leg[i].setGain(ANGLE, kAng+0.1);
    leg[i].setGain(EXTENSION, kExt);
  }
  leg[0].setGain(EXTENSION, kExt/4);

  for(int i=0; i<4;++i){
    leg[i].setPosition(ANGLE, standAng);
    leg[i].setPosition(EXTENSION, standExt);
  }
  leg[3].setPosition(EXTENSION, standExt/2);
  
  standExt = 2;  
  
  if (time<tReady){
      leg[0].setPosition(ANGLE, standAng + (float)time*(sweep)/(float)(tReady-0));
      leg[0].setPosition(EXTENSION, standExt + lift);
  }
  else if(time>=(tReady) && time<(tReady+tLower)){
      leg[0].setPosition(ANGLE, standAng + sweep);
      leg[0].setPosition(EXTENSION, (standExt+lift) + (float)(time-tReady)*(-lift)/(float)(tLower));
  }
  else if(time>=(tReady+tLower) && time<(tReady+tLower+tSweep)){
      leg[0].setPosition(ANGLE, (standAng+sweep) + (float)(time-(tReady+tLower))*(-2*sweep)/(float)(tSweep));
      leg[0].setPosition(EXTENSION, standExt);
  }
  else if(time>=(tReady+tLower+tSweep) && time<(tReady+tLower+tSweep+tRaise)){
      leg[0].setPosition(ANGLE, standAng - sweep);
      leg[0].setPosition(EXTENSION, (standExt) + (float)(time-(tReady+tLower+tSweep))*(lift)/(float)(tRaise));
  }
  else{
      leg[0].setPosition(ANGLE, (standAng - sweep) + (float)(time-(tReady+tLower+tSweep+tRaise))*(sweep)/(float)(tReturn));
      leg[0].setPosition(EXTENSION, standExt + lift); 
  }
  */

//  
//  Serial1.print(time);    
//  Serial1.print('\n');

}


