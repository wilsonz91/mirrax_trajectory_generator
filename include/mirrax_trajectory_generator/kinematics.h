#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

// Enum for Re^6 robot base state vector
enum StateIdentifiers {
    SX = 0
  , SY
  , SQR
  , SQY
  , SJ5
  , SJ6
};

// Enum for Re^3 arm state vector (1 active, 2 passive)
enum ArmStateIdentifiers {
    SJ7 = 0
  , SJ8
  , SJ9
};

// Enum for Re^9 active and passive joint state vector
enum JointIdentifiers {
    W1 = 0
  , W2
  , W3
  , W4
  , J5
  , J6
  , J7
  , J8
  , J9
};

namespace mirrax{

class Kinematics
{
 public:
  Kinematics();
  Kinematics(ros::NodeHandle *nh);
  ~Kinematics();

  std::string robot_name_;  // robot name (mirrax or urax)
  std::string frame_;       // controller frame

  float WHEEL_GR;         // gear ratio from actuator to wheel
  float LEG_GR;           // gear ratio from actuator to leg joints
  float WHEEL_RADIUS;     // wheel radius
  float ROLLER_RADIUS;    // roller radius
  float base_X_wheel_x;   // absolute x-distance from base_link to leg joint
  float base_X_wheel_y1;  // absolute y-distance from base_link to inside wheel
  float base_X_wheel_y2;  // absolute y-distance from base_link to outside wheel

  Eigen::MatrixXd robot_jacobian_;    // Geometric jacobian at robot link
  Eigen::MatrixXd base_jacobian_;     // Geometric jacobian at base link
  Eigen::Vector3d base_X_middlelink;  // Position vector from robot_link to base_link

  // Computes the active linear extension arm angle and pivot arm angle 
  // relative to base_link depending on linear actuator extension
  float calculateLinearActuatorAngle(float l);
	float calculateArmAngle(float l);
  
  // Scales the actuator to wheel velocity according to the 
  // wheel gear ratio and vice versa
  float dxlToWheelVelocity(float v);
  float wheelToDxlVelocity(float v);
  
  // Scales the actuator to leg joint velocity according to the 
  // leg joint gear ratio and vice versa
  float dxlToJointVelocity(float v);
  float jointToDxlVelocity(float v);
  
  // Scales the actuator position to leg joint angle according to 
  // the leg joint gear ratio and vice versa. Also offsets the 
  // joint position when used in multi-turn DXL mode
  float dxlToJointPosition(float p, float offset);
  float jointToDxlPosition(float p, float offset);

  // Sets the kinematic parameters depending on the robot selected
  bool setKinematicParameters(std::string robot_name);

  // Computes the fictitious robot_link position
  void updateBaseXMiddleLink(double j5, double j6);

  // Updates the geometric jacobian wrt robot_link & base_link
  void updateRobotLinkJacobian(double j5, double j6);
  void updateBaseLinkJacobian(double j5, double j6);

  // Inverse the jacobian to compute base_link or robot_link velocity  
  // from wheel and joint velocity. The jacobian, Re^(4,5), can be 
  // thought of as consisting of two parts, J = [Jw, Jj] where 
  // Jw=Re^(4,3) and Jj=Re^(4,2). The base/robot_link velocity, x, 
  // is computed using the following relationship: 
  //      J(q5,q6)*[x dq5 dq6]' = wheel_velocity
  //      [Jw, Jj]*[x dq5 dq6]' = wheel_velocity
  //      Jw*x + Jj*[dq5 dq6]'  = wheel_velocity
  //      x = pinv(Jw)*(wheel_velocity - Jj*[dq5 dq6]')
  void wheelToRobotLinkVelocity(const Eigen::VectorXd& vel, 
    const double j5, const double j6, Eigen::VectorXd& vout);
  void wheelToBaseLinkVelocity(const Eigen::VectorXd& vel, 
    const double j5, const double j6, Eigen::VectorXd& vout);

  bool setFrame(std::string frame);

  // Wrapper for updateJacobian and wheelToTwist, selects the 
  // function depending on the frame used
  void updateJacobian(double j5, double j6);
  Eigen::MatrixXd getJacobian();
  void wheelToTwistVelocity(const Eigen::VectorXd& vel, 
    const double j5, const double j6, Eigen::VectorXd& vout);
  Eigen::VectorXd twistToWheelVelocity(const Eigen::VectorXd& vcmd, 
    const double j5, const double j6);

 private:
  ros::NodeHandle n_;

  double p_j5Xw1_x = 0;
  double p_j5Xw2_x = 0;
  double p_j6Xw3_x = 0;
  double p_j6Xw4_x = 0;
  
};

}; // namespace mirrax

#endif  // KINEMATICS_H_