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
#include <dart/gui/gui.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "MinitaurWorldNode.hpp"
#include "MinitaurEventHandler.hpp"
#include "MinitaurWidget.hpp"

using namespace dart::dynamics;
using namespace dart::simulation;

const double default_height = 0.4; // m
const double default_width = 0.0079;  // m
const double default_depth = 0.0079;  // m

const double default_rest_position = 0.0;
const double delta_rest_position = 10.0 * M_PI / 180.0;

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double shaft_mass = 0.005;
const double mass_mass = 0.5;
const double shaft_length = 0.4;

const double default_damping = 0.0;
const double delta_damping = 0.0;

void setTailShaftGeometry(const BodyNodePtr& bn)
{
  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(
      default_width, shaft_length));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Set the location of the shape node
  Eigen::Isometry3d cyl_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, -shaft_length / 2.0);
  cyl_tf.translation() = center;
  shapeNode->setRelativeTransform(cyl_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);

  Inertia inertia;
  inertia.setMass(shaft_mass);
  inertia.setMoment(cyl->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);
}

BodyNode* addTailShaft(const SkeletonPtr& pendulum, BodyNode* parent,
                  const std::string& name)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mAxis = Eigen::Vector3d::UnitY();
  properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(0, 0, -0.025);
  properties.mT_ParentBodyToJoint.linear() =
      dart::math::eulerXYZToMatrix(Eigen::Vector3d(-90.0 * M_PI / 180.0, 0, 0));
  // properties.mRestPositions[0] = default_rest_position;
  // properties.mSpringStiffnesses[0] = default_stiffness;
  // properties.mDampingCoefficients[0] = default_damping;

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<WeldJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  const double R = default_width / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Line up the cylinder with the Joint axis
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = dart::math::eulerXYZToMatrix(
        Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  setTailShaftGeometry(bn);

  return bn;
}

void setTailMassGeometry(const BodyNodePtr& bn)
{
  // Create a CylinderShape to be used for both visualization and collision checking
  const double R = default_width * 3;
  const double h = default_depth * 6;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Gray());

  // Set the location of the shape node
  Eigen::Isometry3d cyl_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
  cyl_tf.translation() = center;
  shapeNode->setRelativeTransform(cyl_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);

  Inertia inertia;
  inertia.setMass(mass_mass);
  inertia.setMoment(cyl->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);
}

BodyNode* addTailMass(const SkeletonPtr& pendulum, BodyNode* parent,
                  const std::string& name)
{
  // Set up the properties for the Joint
  WeldJoint::Properties properties;
  properties.mName = name + "_joint";
    properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(0 , 0, -shaft_length);

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<WeldJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  const double R = default_width / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Line up the cylinder with the Joint axis
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = dart::math::eulerXYZToMatrix(
        Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  setTailMassGeometry(bn);

  return bn;
}


int main(int argc, char* argv[])
{

  dart::simulation::WorldPtr world(new dart::simulation::World());
  
  // Load ground and Atlas robot and add them to the world
  dart::utils::DartLoader loader;
  auto minitaur = loader.parseSkeleton("dart://sample/urdf/minitaur/quadrupedPrismaticOdie.urdf");
  auto motor = loader.parseSkeleton("dart://sample/urdf/minitaur/motorunit.urdf");
  auto ground = loader.parseSkeleton("dart://sample/urdf/minitaur/ground.urdf");


  // auto chassis = minitaur->getBodyNode("base_chassis_link");
  // auto tailRotor = minitaur->getBodyNode("rotor_tail");
  // BodyNode* tailShaft = addTailShaft(minitaur, tailRotor, "tailShaft");
  // BodyNode* tailMass = addTailMass(minitaur, tailShaft, "tailMass");

  // minitaur->getBodyNode("tailShaft")->getParentJoint()->getDof(0)->setPosition(1);

  world->addSkeleton(motor);
  world->addSkeleton(ground);

  // Set initial configuration for Minitaur robot
  using namespace dart::math::suffixes;
  // minitaur->setPosition(0, 180_deg);
  // minitaur->setPosition(0, 0);
  // minitaur->setPosition(5, 0.3);

  motor->setPosition(0, 0);
  motor->setPosition(5, 0.5);
  motor->setPosition(4, 0);

 // auto frontLR = minitaur->getBodyNode("lower_leg_front_leftR_link");
 // auto frontRL = minitaur->getBodyNode("lower_leg_front_rightL_link");
 // auto backLR = minitaur->getBodyNode("lower_leg_back_leftR_link");
 // auto backRL = minitaur->getBodyNode("lower_leg_back_rightL_link");
 // minitaur->getJoint("knee_front_leftR_link")->getDof(0)->setPositionLimits(0.11,0.29);
 // minitaur->getJoint("knee_front_leftR_link")->setPositionLimitEnforced(true);
 // minitaur->getJoint("knee_back_leftR_link")->getDof(0)->setPositionLimits(0.11,0.29);
 // minitaur->getJoint("knee_back_leftR_link")->setPositionLimitEnforced(true);
 // minitaur->getJoint("knee_front_rightL_link")->getDof(0)->setPositionLimits(0.11,0.29);
 // minitaur->getJoint("knee_front_rightL_link")->setPositionLimitEnforced(true);
 // minitaur->getJoint("knee_back_rightL_link")->getDof(0)->setPositionLimits(0.11,0.29);
 // minitaur->getJoint("knee_back_rightL_link")->setPositionLimitEnforced(true);

 // Eigen::Vector3d offset(0.0, 0.0, -0.2);
 // auto constFrontLeft = std::make_shared<dart::constraint::BallJointConstraint>(
 //     frontLL, frontLR, frontLL->getTransform() * offset);
 // auto constFrontRight = std::make_shared<dart::constraint::BallJointConstraint>(
 //     frontRL, frontRR, frontRL->getTransform() * offset);
 // auto constBackLeft = std::make_shared<dart::constraint::BallJointConstraint>(
 //     backLL, backLR, backLL->getTransform() * offset);
 // auto constBackRight = std::make_shared<dart::constraint::BallJointConstraint>(
 //     backRL, backRR, backRL->getTransform() * offset);
 // // world->getConstraintSolver()->addConstraint(constFrontLeft);
 // // world->getConstraintSolver()->addConstraint(constFrontRight);
 // // world->getConstraintSolver()->addConstraint(constBackLeft);
 // // world->getConstraintSolver()->addConstraint(constBackRight);

  // double IC[] = {PI,0,PI,0,0,PI,0,PI};
  // double IC[] = {0,PI,0,PI,PI,0,PI,0,0.2};
  // double IC[] = {0,0,0,0,0,0,0,0,0.2};
  // double delta = 0.01;
  // minitaur->getJoint("motor_front_leftL_joint")->setPosition(0,IC[0] + delta);
  // minitaur->getJoint("motor_front_leftR_joint")->setPosition(0,IC[1] + delta);
  // minitaur->getJoint("motor_back_leftL_joint")->setPosition(0,IC[2] + delta);
  // minitaur->getJoint("motor_back_leftR_joint")->setPosition(0,IC[3] + delta);
  // minitaur->getJoint("motor_front_rightL_joint")->setPosition(0,IC[4] + delta);
  // minitaur->getJoint("motor_front_rightR_joint")->setPosition(0,IC[5] + delta);
  // minitaur->getJoint("motor_back_rightL_joint")->setPosition(0,IC[6] + delta);
  // minitaur->getJoint("motor_back_rightR_joint")->setPosition(0,IC[7] + delta);
  // minitaur->getJoint("knee_front_leftR_link")->setPosition(0,IC[8]);
  // minitaur->getJoint("knee_back_leftR_link")->setPosition(0,IC[8]);
  // minitaur->getJoint("knee_front_rightL_link")->setPosition(0,IC[8]);
  // minitaur->getJoint("knee_back_rightL_link")->setPosition(0,IC[8]);

  // Set gravity of the world
  // world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setGravity(Eigen::Vector3d(0.0, 0.0, 0));
  // Wrap a WorldNode around it
  osg::ref_ptr<MinitaurWorldNode> node
      = new MinitaurWorldNode(world, motor);
  node->setNumStepsPerCycle(20);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Add control widget for atlas
  viewer.getImGuiHandler()->addWidget(
        std::make_shared<MinitaurWidget>(&viewer, node.get()));

  // Pass in the custom event handler
  viewer.addEventHandler(new MinitaurEventHandler(node));

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set the window name
  viewer.realize();
  osgViewer::Viewer::Windows windows;
  viewer.getWindows(windows);
  windows.front()->setWindowName("Ghost Robotics - Minitaur");

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 2.0, 2.0, 2.0),
        ::osg::Vec3d( 0.0, 0.0, 0.0),
        ::osg::Vec3d( 0.0, 0.0, 1.0));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}
