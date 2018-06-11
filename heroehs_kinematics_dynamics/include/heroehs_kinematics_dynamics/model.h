/*
 * model.h
 *
 *  Created on: 2018. 4. 27.
 *      Author: Crowban
 */

#ifndef HEROEHS_KINEMATICS_DYNAMICS_MODEL_H_
#define HEROEHS_KINEMATICS_DYNAMICS_MODEL_H_

#include "robotis_math/robotis_math.h"

namespace heroehs
{

class LinkData
{
public:
  LinkData();
  LinkData(std::string& name,
      int parent, int child, int sibling,
      double joint_limit_max, double joint_limit_min,
      Eigen::Vector3d& relative_position,
      Eigen::Vector3d& joint_axis,
      Eigen::Vector3d& local_center_of_mass,
      Eigen::Matrix3d& inertia);
  ~LinkData();

  std::string name_;

  int parent_;
  int sibling_;
  int child_;

  Eigen::Vector3d relative_position_;
  Eigen::Vector3d joint_axis_;
  Eigen::Vector3d local_center_of_mass_;
  Eigen::Matrix3d inertia_;
  Eigen::Vector3d current_center_of_mass_;

  double joint_limit_max_;
  double joint_limit_min_;

  double joint_angle_;
  double joint_velocity_;
  double joint_acceleration_;

  Eigen::Vector3d position_;
  Eigen::Matrix3d orientation_;
  Eigen::Matrix4d transformation_;
};

class RobotModel
{
public:
  RobotModel();
  ~RobotModel();

  void addLinkData(LinkData& link_data);
  std::vector<LinkData>& getRobotModel();

private:
  std::vector<LinkData> robot_model;

};

}



#endif /* HEROEHS_KINEMATICS_DYNAMICS_MODEL_H_ */
