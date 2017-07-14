/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "MinitaurWorldNode.hpp"
#include "MinitaurEventHandler.hpp"
#include "MinitaurWidget.hpp"

#include "MyWindow.hpp"

double computeCost(const double *x, const int N)
{
  dart::simulation::WorldPtr world(new dart::simulation::World());
  
  // Load ground and Atlas robot and add them to the world
  dart::utils::DartLoader loader;
  auto minitaur = loader.parseSkeleton("dart://sample/urdf/minitaur/quadruped2Odie.urdf");
  auto ground = loader.parseSkeleton("dart://sample/urdf/minitaur/ground.urdf");

  world->addSkeleton(minitaur);
  world->addSkeleton(ground);

  // Set up simulation time parameters
  double dt = 0.001;
  world->setTimeStep(dt);
  int tFinal = 1;

  // Set initial configuration for Minitaur robot
  using namespace dart::math::suffixes;
  minitaur->setPosition(0, 180_deg);
  minitaur->setPosition(5, 0.5);


  // Set gravity of the world
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  // // Uncomment this to set up for rendering
  // MyWindow window;
  // window.setWorld(world);
  
  // glutInit(&argc, argv);
  // window.initWindow(1280, 960, "GR Minitaur");

  // get initial x position
  auto chassis = minitaur->getBodyNode("base_chassis_link");
  Eigen::Vector3d chassisPosInitial = chassis->getWorldTransform().translation();

  // Motor model parameters
  double P = 0; // initialization
  double kt = 0.0954;
  double kv = 16;
  double Nr = 1;
  double Ra = 0.186;

    // Wrap a WorldNode around it
  osg::ref_ptr<MinitaurWorldNode> node
      = new MinitaurWorldNode(world, minitaur);
  node->setNumStepsPerCycle(20);
  
  // Simulate
  for(int i = 0; i<=tFinal/dt; ++i){
    node->customPreStep();
    world->step();
    for(int j = 0;j<8;++j){
      P += dt*(DARTMotorCommand[j]*DARTMotorCommand[j]*Ra/(Nr*Nr*kt*kt) + DARTMotorCommand[j]*DARTMotorVel[j]*kv/kt);
    }
    //window.drawWorld();
    // dstd::cout << "Step " << i << std::endl;
  }
  //glutMainLoop();

  // get final position to calculate distance
  Eigen::Vector3d chassisPosFinal = chassis->getWorldTransform().translation();
  double distance = chassisPosFinal[0] - chassisPosInitial[0];
  return P/(distance*5.494*9.81);

}
