/*
 * kinematics.h
 *
 *  Created on: 2018. 4. 27.
 *      Author: Crowban
 */

#ifndef HEROEHS_KINEMATICS_DYNAMICS_KINEMATICS_H_
#define HEROEHS_KINEMATICS_DYNAMICS_KINEMATICS_H_

#include "heroehs_kinematics_dynamics/model.h"

namespace heroehs
{

class KinematicsSolver
{
public:
  KinematicsSolver(RobotModel *model);
  virtual ~KinematicsSolver();

  void initialize();
  void calcForwardKinematics(int start_link_idx = 0);

  double calcTotalMass(int start_link_idx = 0);
  void calcJointCenterOfMass(int joint_id);
  Eigen::Vector3d calcCenterOfMass();


  RobotModel *model;
  double total_mass;

};

class HumanoidKinematicsSolver : public heroehs::KinematicsSolver
{
public:
  HumanoidKinematicsSolver(RobotModel *model);
  virtual ~HumanoidKinematicsSolver();

  void calcForwardKinematics(int start_link_idx = 0);

  virtual bool calcInverseKinematicsForLeg(double *right_out, double *left_out, Eigen::Matrix4d rhip_to_rfoot, Eigen::Matrix4d lhip_to_lfoot) = 0;
  virtual bool calcInverseKinematicsForRightLeg(double *out, Eigen::Matrix4d rhip_to_rfoot, ) = 0;
  virtual bool calcInverseKinematicsForLeftLeg(double *out, Eigen::Matrix4d lhip_to_lfoot)  = 0;

private:

};

}



#endif /* HEROEHS_KINEMATICS_DYNAMICS_KINEMATICS_H_ */
