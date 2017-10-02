/*
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "MinitaurWorldNode.hpp"

float DARTMotorPos[8];
float DARTMotorVel[8];
float DARTTime;
float DARTIMUData[6];

//Interface interface;

//==============================================================================
MinitaurWorldNode::MinitaurWorldNode(
    const dart::simulation::WorldPtr& world,
    const dart::dynamics::SkeletonPtr& atlas)
  : dart::gui::osg::WorldNode(world),
    mExternalForce(Eigen::Vector3d::Zero()),
    mForceDuration(0.0)
{

  assert(world);
  assert(atlas);

  interface.setup();
}

//==============================================================================
void MinitaurWorldNode::customPreStep()
{
  auto chassis = mWorld->getSkeleton(0)->getBodyNode("base_chassis_link");
  chassis->addExtForce(mExternalForce);
  //mController->update();
  std::array<dart::dynamics::Joint*,4> angles;
  std::array<dart::dynamics::Joint*,4> extensions;
  float toePos[2];
  float motorCommands[2];
  float dartCommands[2];
  float l1 = 0.1; //m
  float l2 = 0.2; //m


  angles[0] = mWorld->getSkeleton(0)->getJoint("motor_front_leftR_joint");
  angles[1] = mWorld->getSkeleton(0)->getJoint("motor_back_leftR_joint");
  angles[2] = mWorld->getSkeleton(0)->getJoint("motor_front_rightL_joint");
  angles[3] = mWorld->getSkeleton(0)->getJoint("motor_back_rightL_joint");
  extensions[0] = mWorld->getSkeleton(0)->getJoint("knee_front_leftR_link");
  extensions[1] = mWorld->getSkeleton(0)->getJoint("knee_back_leftR_link");
  extensions[2] = mWorld->getSkeleton(0)->getJoint("knee_front_rightL_link");
  extensions[3] = mWorld->getSkeleton(0)->getJoint("knee_back_rightL_link");

  for(int i=0;i<4;++i){
    toePos[0] = extensions[i]->getDof(0)->getPosition();
    toePos[1] = dart::math::wrapToPi(angles[i]->getDof(0)->getPosition());

    float diffAng = PI - acosf((l1*l1 - l2*l2 + toePos[0]*toePos[0])/(2*l1*toePos[0]));

    // Invert the mean/diff coordinate change

    DARTMotorPos[2*i+1] = dart::math::wrapToPi(diffAng + ((i<2) ? toePos[1] : (-toePos[1])));
    DARTMotorPos[2*i] = dart::math::wrapToPi(diffAng - ((i<2) ? toePos[1] : (-toePos[1])));
  }

  // for(int i=0;i<8;++i){
  //   DARTMotorPos[i] = angles[i]->getDof(0)->getPosition();
  //   DARTMotorVel[i] = angles[i]->getDof(0)->getVelocity();
  // }
  DARTTime = mWorld->getTime();
  std::cout << "t = " << DARTTime << " s" << std::endl;

  Eigen::Matrix3d R = chassis->getWorldTransform().linear();
  Eigen::Vector3d rpy = dart::math::matrixToEulerXYZ(R);
  rpy[0] = dart::math::wrapToPi(rpy[0] + M_PI);
  Eigen::Vector3d w_w = chassis->getAngularVelocity();
  Eigen::Matrix3d invJ;
  invJ << 1,sin(rpy[1])*sin(rpy[0])/cos(rpy[1]),cos(rpy[0])*sin(rpy[1])/cos(rpy[1]),0,cos(rpy[0]),-sin(rpy[0]),0,sin(rpy[0])/cos(rpy[1]),cos(rpy[0])/cos(rpy[1]);
  Eigen::Vector3d drpy = invJ * R.transpose() * w_w;
  for(int i = 0;i<3;++i){
    DARTIMUData[i] = rpy[i];
    DARTIMUData[i+3] = drpy[i];
  }

  interface.update();

  for(int i = 0;i<8;++i){
    std::cout << "DARTMotorPos: " << DARTMotorPos[i] << std::endl;
    std::cout << "getPosition (ANGLE): " << leg[i/2].getPosition(ANGLE) << std::endl;
  }


  for(int i = 0;i<4;++i){
    
    // motorCommands[(i<2) ? 1 : 0] = DARTMotorCommand[2*i];
    // motorCommands[(i<2) ? 0 : 1] = DARTMotorCommand[2*i+1];
    // leg[i].useLengths = true;
    // leg[i].physicalToAbstract(motorCommands, dartCommands);
    // leg[i].useLengths = false;
    // float extDartCommand = dartCommands[0];
    // float angDartCommand = dartCommands[1];


    motorCommands[(i<2) ? 1 : 0] = DARTMotorCommand[2*i];
    motorCommands[(i<2) ? 0 : 1] = DARTMotorCommand[2*i+1];

    float thetaOut = DARTMotorPos[2*i + ((i<2) ? 0 : 1)];
    float thetaIn  = DARTMotorPos[2*i + ((i<2) ? 1 : 0)];
    float meanAng = 0.5*(thetaOut - thetaIn);
    float diffAng = 0.5*(thetaOut + thetaIn);
    float meanTorque = (motorCommands[0] - motorCommands[1])*0.5;
    float diffTorque = (motorCommands[0] + motorCommands[1])*0.5;

    float angDartCommand = meanTorque;
    float extDartCommand = (l1*sin(diffAng) + (-l1*cos(diffAng)*l1*sin(diffAng)/sqrt(l2*l2 - pow(l1*sin(diffAng),2))))*diffTorque;


    //std::cout << DARTMotorCommand[i] << "\t";//DARTMotorCommand[i] = 100.0; //This will be replaced with actual commands from DART
    extensions[i]->getDof(0)->setForce(extDartCommand);
    angles[i]->getDof(0)->setForce(angDartCommand);

    std::cout << "Leg " << i << ":" << std::endl;
    std::cout << "Motor Commands (out, in): " << motorCommands[1] << ", " << motorCommands[0] << std::endl;
    std::cout << "Leg Commands (angle, extension): " << angDartCommand << ", " << extDartCommand << std::endl;
    // std::cout << "Inside sqrt: " << l2*l2 - pow(l1*sin(diffAng),2) << std::endl;
    // std::cout << "diffAng: " << diffAng << std::endl;
    // std::cout << "meanAng: " << meanAng << std::endl;
    std::cout << "Ext: " << extDartCommand << std::endl;
    std::cout << "Ang: " << angDartCommand << std::endl;
  }

  // std::cout << "DARTMotor Pos " << DARTMotorPos[0] << std::endl;
  // std::cout << "GR Motor Raw Pos " << M[0].getRawPosition() << std::endl;
  // std::cout << "GR Motor Pos " << M[0].getPosition() << std::endl;

  // Set new tail force to 0 for testing purposes
  mWorld->getSkeleton(0)->getBodyNode("rotor_tail")->getParentJoint()->getDof(0)->setForce(0);
  // std::cout << rcCmd[0] << "\t" << rcCmd[1] << "\t" <<rcCmd[2] << "\t" <<rcCmd[3] << "\t" <<rcCmd[4] << "\t" <<rcCmd[5] <<std::endl;
  if (mForceDuration > 0)
    mForceDuration--;
  else
    mExternalForce.setZero();
}

//==============================================================================
void MinitaurWorldNode::reset()
{
  mExternalForce.setZero();
}

//==============================================================================
void MinitaurWorldNode::pushForwardAtlas(double force, int frames)
{
  mExternalForce.x() = force;
  mForceDuration = frames;
}

//==============================================================================
void MinitaurWorldNode::pushBackwardAtlas(double force, int frames)
{
  mExternalForce.x() = -force;
  mForceDuration = frames;
}

//==============================================================================
void MinitaurWorldNode::pushLeftAtlas(double force, int frames)
{
  mExternalForce.z() = force;
  mForceDuration = frames;
}

//==============================================================================
void MinitaurWorldNode::pushRightAtlas(double force, int frames)
{
  mExternalForce.z() = -force;
  mForceDuration = frames;
}

//==============================================================================
void MinitaurWorldNode::switchToNormalStrideWalking()
{
}

//==============================================================================
void MinitaurWorldNode::switchToWarmingUp()
{
}

//==============================================================================
void MinitaurWorldNode::switchToNoControl()
{
}
