#include "mirrax_trajectory_generator/kinematics.h"

namespace mirrax{

Kinematics::Kinematics() :
  frame_("robot"),
  robot_name_("mirrax")
{
  n_.param("/robot_name", robot_name_, robot_name_);

  robot_jacobian_.resize(4,5);
  base_jacobian_.resize(4,5);
}

Kinematics::Kinematics(ros::NodeHandle *nh) : 
  n_(*nh),
  frame_("robot"),
  robot_name_("mirrax")
{
  // Set robot kinematic parameters from parameter server
  n_.param("/robot_name", robot_name_, robot_name_);
  n_.param("/mirrax/kinematic/l1", base_X_wheel_x, base_X_wheel_x);
  n_.param("/mirrax/kinematic/l2", base_X_wheel_y2, base_X_wheel_y2);
  n_.param("/mirrax/kinematic/l3", base_X_wheel_y1, base_X_wheel_y1);
  n_.param("/mirrax/kinematic/r_w", WHEEL_RADIUS, WHEEL_RADIUS);
  n_.param("/mirrax/kinematic/r_r", ROLLER_RADIUS, ROLLER_RADIUS);
  n_.param("/mirrax/gear_ratio", WHEEL_GR, WHEEL_GR);
  n_.param("/mirrax/gear_ratio", LEG_GR, LEG_GR);
  n_.param("/mirrax/frame", frame_, frame_);
  
  ROS_INFO("[kinematics] Robot `%s` selected",robot_name_.c_str());

  robot_jacobian_.resize(4,5);
  base_jacobian_.resize(4,5);
}

Kinematics::~Kinematics() {}

bool Kinematics::setFrame(std::string frame)
{
  if (frame_=="robot" || frame_=="base")
    frame_ = frame;
  else
    printf("Expecting 'robot' or 'base' frame, invalid frame %s selected\n",frame.c_str());
  // printf("Frame: %s\n",frame_.c_str());
}

void Kinematics::updateJacobian(double j5, double j6)
{
  if (frame_=="robot")
    updateRobotLinkJacobian(j5,j6);
  else if (frame_=="base")
    updateBaseLinkJacobian(j5,j6);
}

Eigen::MatrixXd Kinematics::getJacobian()
{
  if (frame_=="robot")
    return robot_jacobian_;    
  else if (frame_=="base")
    return base_jacobian_;
}

void Kinematics::wheelToTwistVelocity(const Eigen::VectorXd& vel, 
  const double j5, const double j6, Eigen::VectorXd& vout)
{
  if (frame_=="robot")
    wheelToRobotLinkVelocity(vel, j5, j6, vout);
  else if (frame_=="base")
    wheelToBaseLinkVelocity(vel, j5, j6, vout);
}

Eigen::VectorXd Kinematics::twistToWheelVelocity(const Eigen::VectorXd& vcmd, 
    const double j5, const double j6)
{
  if (frame_=="robot")
  {
    updateRobotLinkJacobian(j5,j6);
    return (robot_jacobian_*vcmd);
  }
  else if (frame_=="base")
  {
    updateBaseLinkJacobian(j5,j6);
    return (base_jacobian_*vcmd);
  }
}

bool Kinematics::setKinematicParameters(std::string robot_name)
{
  if (robot_name == "mirrax")
  {
    std::cout << "Setting to Mirrax" << std::endl;
    WHEEL_GR = 1.0/3;
    LEG_GR = 1.0/3;
    WHEEL_RADIUS = 0.123/2;
    base_X_wheel_x = 0.216;
    base_X_wheel_y1 = 0.4005;
    base_X_wheel_y2 = 0.1215;
    return true;
  }
  else if (robot_name == "urax")
  {
    std::cout << "Setting to Mini-Mirrax" << std::endl;
    WHEEL_GR = 1.0;
    LEG_GR = 1.0;
    WHEEL_RADIUS = 0.05;
    base_X_wheel_x = 0.216/2;
    base_X_wheel_y1 = 0.21225;
    base_X_wheel_y2 = 0.07125;
    return true;
  }
  else
  {
    // std::cout << "Invalid robot " << robot_name << "selected!" << std::endl;
    printf("Invalid robot %s selected!", robot_name.c_str());
    return false;
  }
}

float Kinematics::calculateLinearActuatorAngle(float l)
{ //acos((a^2+b^2-c^2)/2ab)
	return acos(((0.168+l)*(0.168+l)+0.054648-0.004685)/(2*(0.168+l)*0.23377));
}
float Kinematics::calculateArmAngle(float l)
{ //acos((a^2+b^2-c^2)/2ab)
	return 1.917796-acos((0.054648+0.004685-(0.168+l)*(0.168+l))/(2*0.06845*0.23377));
}

float Kinematics::dxlToWheelVelocity(float v)
{
  return v*WHEEL_GR;
}

float Kinematics::wheelToDxlVelocity(float v)
{
  return v*1/WHEEL_GR;
}

float Kinematics::dxlToJointVelocity(float v)
{
  return v*LEG_GR;
}

float Kinematics::jointToDxlVelocity(float v)
{
  return v*1/LEG_GR;
}

float Kinematics::jointToDxlPosition(float p, float offset)
{
  return (p + offset)/LEG_GR;
}

float Kinematics::dxlToJointPosition(float p, float offset)
{
  // std::cout << "Leg gr: " << LEG_GR << std::endl;
  return p*LEG_GR - offset;
}

void Kinematics::updateRobotLinkJacobian(double j5, double j6)
{
  updateBaseXMiddleLink(j5, j6);
  
  // Remap for convenience in exporting from matlab
  double pbXLm_x = base_X_middlelink(0);
  double pbXLm_y = base_X_middlelink(1);
  double r_w = WHEEL_RADIUS;
  double p_LmXj5_x = -base_X_wheel_x;
  double p_LmXj6_x =  base_X_wheel_x; 
  double p_j5Xw1_y = base_X_wheel_y1;
  double p_j5Xw2_y = base_X_wheel_y2;
  double p_j6Xw3_y = base_X_wheel_y2;
  double p_j6Xw4_y = base_X_wheel_y1;

  // Now update jacobian
  robot_jacobian_.row(0) << (1.4142*cos(j5 + 0.7854))/r_w,  (1.4142*sin(j5 + 0.7854))/r_w,  (1.4142*(0.7071*p_j5Xw1_x - 0.7071*p_j5Xw1_y + 0.7071*p_LmXj5_x*cos(j5) + 0.7071*pbXLm_x*cos(j5) - 0.7071*pbXLm_y*cos(j5) + 0.7071*p_LmXj5_x*sin(j5) + 0.7071*pbXLm_x*sin(j5) + 0.7071*pbXLm_y*sin(j5)))/r_w,  (p_j5Xw1_x - p_j5Xw1_y)/r_w,                            0;
  robot_jacobian_.row(1) << (1.4142*sin(j5 + 0.7854))/r_w, -(1.4142*cos(j5 + 0.7854))/r_w, -(1.4142*(0.7071*p_j5Xw2_x + 0.7071*p_j5Xw2_y + 0.7071*p_LmXj5_x*cos(j5) + 0.7071*pbXLm_x*cos(j5) + 0.7071*pbXLm_y*cos(j5) - 0.7071*p_LmXj5_x*sin(j5) - 0.7071*pbXLm_x*sin(j5) + 0.7071*pbXLm_y*sin(j5)))/r_w, -(p_j5Xw2_x + p_j5Xw2_y)/r_w,                            0;
  robot_jacobian_.row(2) << (1.4142*cos(j6 + 0.7854))/r_w,  (1.4142*sin(j6 + 0.7854))/r_w,  (1.4142*(0.7071*p_j6Xw3_x - 0.7071*p_j6Xw3_y + 0.7071*p_LmXj6_x*cos(j6) + 0.7071*pbXLm_x*cos(j6) - 0.7071*pbXLm_y*cos(j6) + 0.7071*p_LmXj6_x*sin(j6) + 0.7071*pbXLm_x*sin(j6) + 0.7071*pbXLm_y*sin(j6)))/r_w,                            0,  (p_j6Xw3_x - p_j6Xw3_y)/r_w;
  robot_jacobian_.row(3) << (1.4142*sin(j6 + 0.7854))/r_w, -(1.4142*cos(j6 + 0.7854))/r_w, -(1.4142*(0.7071*p_j6Xw4_x + 0.7071*p_j6Xw4_y + 0.7071*p_LmXj6_x*cos(j6) + 0.7071*pbXLm_x*cos(j6) + 0.7071*pbXLm_y*cos(j6) - 0.7071*p_LmXj6_x*sin(j6) - 0.7071*pbXLm_x*sin(j6) + 0.7071*pbXLm_y*sin(j6)))/r_w,                            0, -(p_j6Xw4_x + p_j6Xw4_y)/r_w;
  // std::cout << "Jac wheel:\n" << robot_jacobian_ << std::endl;
}

void Kinematics::updateBaseXMiddleLink(double j5, double j6)
{
  double r_w = WHEEL_RADIUS;
  double p_LmXj5_x = -base_X_wheel_x;
  double p_LmXj6_x =  base_X_wheel_x; 
  double p_j5Xw1_y = base_X_wheel_y1;
  double p_j5Xw2_y = base_X_wheel_y2;
  double p_j6Xw3_y = base_X_wheel_y2;
  double p_j6Xw4_y = base_X_wheel_y1;
  
  base_X_middlelink(0) =  0.2500*p_j5Xw1_y*sin(j5) - 0.5000*p_LmXj6_x - 0.2500*p_j5Xw1_x*cos(j5) - 0.2500*p_j5Xw2_x*cos(j5) - 0.2500*p_j6Xw3_x*cos(j6) - 0.2500*p_j6Xw4_x*cos(j6) - 0.5000*p_LmXj5_x + 0.2500*p_j5Xw2_y*sin(j5) + 0.2500*p_j6Xw3_y*sin(j6) + 0.2500*p_j6Xw4_y*sin(j6);
  base_X_middlelink(1) = -0.2500*p_j5Xw1_y*cos(j5) - 0.2500*p_j5Xw2_y*cos(j5) - 0.2500*p_j6Xw3_y*cos(j6) - 0.2500*p_j6Xw4_y*cos(j6) - 0.2500*p_j5Xw1_x*sin(j5) - 0.2500*p_j5Xw2_x*sin(j5) - 0.2500*p_j6Xw3_x*sin(j6) - 0.2500*p_j6Xw4_x*sin(j6);
  // std::cout << "baseXLm: " << base_X_middlelink.transpose() << std::endl;
}

void Kinematics::wheelToRobotLinkVelocity(const Eigen::VectorXd& vel, 
  const double j5, const double j6, Eigen::VectorXd& vout)
{
  // Update robot legs and jacobian
  updateRobotLinkJacobian(j5, j6);
  
  // Remap for (Ax - b) form
  auto b = vel.head(4) - robot_jacobian_.rightCols(2)*vel.tail(2);
  auto A = robot_jacobian_.leftCols(3);

  // Compute x
  vout = A.colPivHouseholderQr().solve(b);
}

void Kinematics::updateBaseLinkJacobian(double j5, double j6)
{
  base_jacobian_.row(0) << 20*cos(j5) - 20*sin(j5), 20*cos(j5) + 20*sin(j5), - 2.1600*cos(j5) - 2.1600*sin(j5) - 4.2250, -4.2250,       0;
  base_jacobian_.row(1) << 20*cos(j5) + 20*sin(j5), 20*sin(j5) - 20*cos(j5),   2.1600*cos(j5) - 2.1600*sin(j5) - 1.4050, -1.4050,       0;
  base_jacobian_.row(2) << 20*cos(j6) - 20*sin(j6), 20*cos(j6) + 20*sin(j6),   2.1600*cos(j6) + 2.1600*sin(j6) - 1.4050,       0, -1.4050;
  base_jacobian_.row(3) << 20*cos(j6) + 20*sin(j6), 20*sin(j6) - 20*cos(j6),   2.1600*sin(j6) - 2.1600*cos(j6) - 4.2250,       0, -4.2250;
}

void Kinematics::wheelToBaseLinkVelocity(const Eigen::VectorXd& vel, 
  const double j5, const double j6, Eigen::VectorXd& vout)
{
  // Update robot legs and jacobian
  updateBaseLinkJacobian(j5, j6);
  
  // Remap for (Ax - b) form
  auto b = vel.head(4) - base_jacobian_.rightCols(2)*vel.tail(2);
  auto A = base_jacobian_.leftCols(3);

  // Compute x
  vout = A.colPivHouseholderQr().solve(b);
}

}