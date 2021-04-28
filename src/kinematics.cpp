#include "mirrax_trajectory_generator/kinematics.h"

namespace mirrax{

Kinematics::Kinematics() 
{
  jac_wheel_.resize(4,5);
}

Kinematics::~Kinematics() {}

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
  else if (robot_name == "mini")
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
  return -v*WHEEL_GR;
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
/*
Eigen::VectorXd Kinematics::robotVelocityToWheelVelocity(
  Vector5d vel, float j5, float j6)
{
	Vector4d wheel_vel;
  for(int i=1; i<=4; i++)
	{
		wheel_vel[i-1] = robotPoseToWheelVelocity(i, j5, j6, vel);
	}
	return wheel_vel;
}

double Kinematics::robotPoseToWheelVelocity(
	int wheel_number, float j5, float j6, Vector5d v)
{
  // ROS_DEBUG("theta j1 %f,  theta j2  %f,  theta_dot j1  %f,  theta_dot j2  %f",j5,j6, theta_dot_J1, theta_dot_J2);

  Eigen::ArrayXd gamma = Eigen::ArrayXd(4) ;
  Eigen::ArrayXd l2 = Eigen::ArrayXd(4);
  Eigen::ArrayXd l3 = Eigen::ArrayXd(4);

  Eigen::ArrayXd theta(4);
  Eigen::ArrayXd theta_dot(4);   
  Eigen::ArrayXd xw_b(4);
  Eigen::ArrayXd yw_b(4);
  Eigen::ArrayXd xw_R(4);
  Eigen::ArrayXd yw_R(4);

  Eigen::MatrixXd cmd(4,1); //transpose
  Eigen::Matrix2d W_inv;
  Eigen::Matrix2d T_inv;
  Eigen::MatrixXd J_theta(2,1);
  Eigen::MatrixXd J_phi(2,1);
  Eigen::MatrixXd Jxy(2,2); 
  Eigen::MatrixXd J(2,4);
  Eigen::MatrixXd Omega_up(2,4);

  gamma << -M_PI/4, M_PI/4, -M_PI/4, M_PI/4; //roller angle   
  // l2 << -0.216, -0.216, 0.216, 0.216; // body to joint frame linear transform in X_B
  // l3 << 0.4005, 0.1215, 0.1215, 0.4005;
  l2 << -base_X_wheel_x, -base_X_wheel_x, base_X_wheel_x, base_X_wheel_x; // body to joint frame linear transform in X_B
  l3 << base_X_wheel_y1, base_X_wheel_y2, base_X_wheel_y2, base_X_wheel_y1;
  
  // CONVERSION FROM KEIR'S FRAME TO HORATIO'S FRAME : ToDo: Translate this into URDF
  // See mirrax_kinematics git repo  - this is just for the kinematics_test node
  // j5 = j5 + M_PI/2;
  // j6 = j6 - M_PI/2;  

  // ROS_DEBUG("J1: %f , J2: %f", j5, j6);

  theta_dot << v(3), v(3), v(4), v(4);   
  theta << j5, j5, j6, j6;

  xw_b = -1*(l3*sin(theta)) + l2;
  yw_b = l3*cos(theta);

  // // from XRYR to XBYB
  double X_RB =  -1*(xw_b.sum())/xw_b.size(); //mean *-1
  double Y_RB =  -1*(yw_b.sum())/yw_b.size();

  // // from XRYR
  xw_R = xw_b + X_RB;
  yw_R = yw_b + Y_RB;
  
  cmd << v(0), v(1), v(2), theta_dot[wheel_number-1] ; 

  float cosec_gamma = 1/(sin(gamma[wheel_number-1]));
  
  W_inv << 1/WHEEL_RADIUS, -tan(gamma[wheel_number-1])/WHEEL_RADIUS ,
                0    , cosec_gamma ;

  T_inv << cos(theta[wheel_number-1]) , sin(theta[wheel_number-1]),
          -1*sin(theta[wheel_number-1]) , cos(theta[wheel_number-1]);

  Jxy <<  1,0,
          0,1;

  J_theta << -1*l3[wheel_number-1]*cos(theta[wheel_number-1]),
              -1*l3[wheel_number-1]*sin(theta[wheel_number-1]);

  J_phi << -1*yw_R[wheel_number-1],
            xw_R[wheel_number-1];

  J << Jxy, J_phi, J_theta ;

  Omega_up  = W_inv * T_inv * J * cmd;

  double driven = Omega_up(0);

  return driven; 
}
*/
void Kinematics::updateKinematics(double j5, double j6)
{
  // setKinematicParameters("mini");
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
  jac_wheel_.row(0) << (1.4142*cos(j5 + 0.7854))/r_w,  (1.4142*sin(j5 + 0.7854))/r_w,  (1.4142*(0.7071*p_j5Xw1_x - 0.7071*p_j5Xw1_y + 0.7071*p_LmXj5_x*cos(j5) + 0.7071*pbXLm_x*cos(j5) - 0.7071*pbXLm_y*cos(j5) + 0.7071*p_LmXj5_x*sin(j5) + 0.7071*pbXLm_x*sin(j5) + 0.7071*pbXLm_y*sin(j5)))/r_w,  (p_j5Xw1_x - p_j5Xw1_y)/r_w,                            0;
  jac_wheel_.row(1) << (1.4142*sin(j5 + 0.7854))/r_w, -(1.4142*cos(j5 + 0.7854))/r_w, -(1.4142*(0.7071*p_j5Xw2_x + 0.7071*p_j5Xw2_y + 0.7071*p_LmXj5_x*cos(j5) + 0.7071*pbXLm_x*cos(j5) + 0.7071*pbXLm_y*cos(j5) - 0.7071*p_LmXj5_x*sin(j5) - 0.7071*pbXLm_x*sin(j5) + 0.7071*pbXLm_y*sin(j5)))/r_w, -(p_j5Xw2_x + p_j5Xw2_y)/r_w,                            0;
  jac_wheel_.row(2) << (1.4142*cos(j6 + 0.7854))/r_w,  (1.4142*sin(j6 + 0.7854))/r_w,  (1.4142*(0.7071*p_j6Xw3_x - 0.7071*p_j6Xw3_y + 0.7071*p_LmXj6_x*cos(j6) + 0.7071*pbXLm_x*cos(j6) - 0.7071*pbXLm_y*cos(j6) + 0.7071*p_LmXj6_x*sin(j6) + 0.7071*pbXLm_x*sin(j6) + 0.7071*pbXLm_y*sin(j6)))/r_w,                            0,  (p_j6Xw3_x - p_j6Xw3_y)/r_w;
  jac_wheel_.row(3) << (1.4142*sin(j6 + 0.7854))/r_w, -(1.4142*cos(j6 + 0.7854))/r_w, -(1.4142*(0.7071*p_j6Xw4_x + 0.7071*p_j6Xw4_y + 0.7071*p_LmXj6_x*cos(j6) + 0.7071*pbXLm_x*cos(j6) + 0.7071*pbXLm_y*cos(j6) - 0.7071*p_LmXj6_x*sin(j6) - 0.7071*pbXLm_x*sin(j6) + 0.7071*pbXLm_y*sin(j6)))/r_w,                            0, -(p_j6Xw4_x + p_j6Xw4_y)/r_w;
  // std::cout << "Jac wheel:\n" << jac_wheel_ << std::endl;
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

}