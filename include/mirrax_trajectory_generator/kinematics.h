#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <vector>
#include <iostream>
#include <Eigen/Dense>

namespace mirrax{

class Kinematics
{
 public:
  Kinematics();
  ~Kinematics();

  float WHEEL_GR;
  float LEG_GR;
  float WHEEL_RADIUS;
  float base_X_wheel_x;
  float base_X_wheel_y1;
  float base_X_wheel_y2;

  Eigen::MatrixXd jac_wheel_;
  Eigen::Vector3d base_X_middlelink;

  float calculateLinearActuatorAngle(float l);
	float calculateArmAngle(float l);
  float dxlToWheelVelocity(float v);
  float wheelToDxlVelocity(float v);
  float dxlToJointVelocity(float v);
  float jointToDxlVelocity(float v);
  float dxlToJointPosition(float p, float offset);
  float jointToDxlPosition(float p, float offset);

  // double robotPoseToWheelVelocity(int , float ,
  //   float , Vector5d );
  
  // Eigen::VectorXd robotVelocityToWheelVelocity(
  //   Vector5d vel, float j5, float j6);

  bool setKinematicParameters(std::string robot_name);

  void updateKinematics(double j5, double j6);
  void updateBaseXMiddleLink(double j5, double j6);

 private:
  double p_j5Xw1_x = 0;
  double p_j5Xw2_x = 0;
  double p_j6Xw3_x = 0;
  double p_j6Xw4_x = 0;
  
};

}; // namespace mirrax

#endif  // KINEMATICS_H_