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

  mController.reset(new Controller(atlas, world->getConstraintSolver()));

  interface.setup();
}

//==============================================================================
void MinitaurWorldNode::customPreStep()
{
  auto chassis = mController->getAtlasRobot()->getBodyNode("base_chassis_link");
  chassis->addExtForce(mExternalForce);
  //mController->update();
  std::array<dart::dynamics::Joint*,8> hips;
  hips[0] = mWorld->getSkeleton(0)->getJoint("motor_front_rightR_joint");
  hips[1] = mWorld->getSkeleton(0)->getJoint("motor_front_rightL_joint");
  hips[2] = mWorld->getSkeleton(0)->getJoint("motor_back_rightR_joint");
  hips[3] = mWorld->getSkeleton(0)->getJoint("motor_back_rightL_joint");
  hips[4] = mWorld->getSkeleton(0)->getJoint("motor_front_leftL_joint");
  hips[5] = mWorld->getSkeleton(0)->getJoint("motor_front_leftR_joint");
  hips[6] = mWorld->getSkeleton(0)->getJoint("motor_back_leftL_joint");
  hips[7] = mWorld->getSkeleton(0)->getJoint("motor_back_leftR_joint");


  for(int i=0;i<8;++i){
    DARTMotorPos[i] = hips[i]->getDof(0)->getPosition();
    DARTMotorVel[i] = hips[i]->getDof(0)->getVelocity();
  }
  DARTTime = mWorld->getTime();

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
  //std::cout << "customPreStep?" << std::endl;
  interface.update();

  for(int i = 0;i<8;++i){
    //std::cout << DARTMotorCommand[i] << "\t";//DARTMotorCommand[i] = 100.0; //This will be replaced with actual commands from DART
    hips[i]->getDof(0)->setForce(DARTMotorCommand[i]);
  }
  //std::cout << rcCmd[0] << "\t" << rcCmd[1] << "\t" <<rcCmd[2] << "\t" <<rcCmd[3] << "\t" <<rcCmd[4] << "\t" <<rcCmd[5] <<std::endl;

  if (mForceDuration > 0)
    mForceDuration--;
  else
    mExternalForce.setZero();
}

//==============================================================================
void MinitaurWorldNode::reset()
{
  mExternalForce.setZero();
  mController->resetRobot();
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
  mController->changeStateMachine("walking", mWorld->getTime());
}

//==============================================================================
void MinitaurWorldNode::switchToWarmingUp()
{
  mController->changeStateMachine("warming up", mWorld->getTime());
}

//==============================================================================
void MinitaurWorldNode::switchToNoControl()
{
  mController->changeStateMachine("standing", mWorld->getTime());
}
