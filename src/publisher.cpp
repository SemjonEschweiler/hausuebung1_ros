#include "ros/ros.h"
#include "std_msgs/String.h"
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <math.h>


float deg2rad(float degrees){
  float rad;
  rad = degrees / 180 * M_PI;
  return rad;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");  
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("robot", 10);

  int hz = 10;
  ros::Rate loop_rate(hz);

  float baseline, diameter, delta_x, delta_y, delta_theta;
  n.getParam("baseline", baseline);
  n.getParam("diameter", diameter);
  n.getParam("delta_x", delta_x);
  n.getParam("delta_y", delta_y);
  n.getParam("delta_theta", delta_theta);
  float l_l, r, theta, alpha_l, alpha_r, beta_r, l_r, beta_l, phi_l, phi_r;
 
  l_l = baseline/2;
  l_r = baseline/2;
  alpha_l = deg2rad(90);
  alpha_r = deg2rad(-90);
  beta_l = deg2rad(0);
  beta_r = deg2rad(180);
  theta = atan2(delta_y, delta_x);
  r = diameter/2;
  ROS_INFO_STREAM("alpha_l: " << alpha_l);
  ROS_INFO_STREAM("baseline: " << baseline);
  ROS_INFO_STREAM("delta_x: " << delta_x);
  ROS_INFO_STREAM("delta_y: " << delta_y);
  ROS_INFO_STREAM("delta_theta: " << delta_theta);
  ROS_INFO_STREAM("diameter: " << diameter);
  ROS_INFO_STREAM("r: " << r);

  ROS_INFO_STREAM("theta: " << theta);

  Eigen::Matrix <float, 1, 3> R_star_l, R_star_r;
  R_star_l << sin(alpha_l + beta_l), -cos(alpha_l + beta_l), -l_l*cos(beta_l);
  R_star_r << sin(alpha_r + beta_r), -cos(alpha_r + beta_r), -l_r*cos(beta_r);
  ROS_INFO_STREAM("R_star_l: \n" << R_star_l);
  ROS_INFO_STREAM("R_star_l: \n" << R_star_l);
  
  Eigen::Matrix <float, 3, 3> RotationMatrix;
  RotationMatrix << cos(theta), sin(theta), 0,
    -sin(theta), cos(theta),0,
    0,0,1; 
  ROS_INFO_STREAM("RotationMatrix: \n" << RotationMatrix);

  Eigen::Matrix <float, 3, 1> Xi_W;
  Xi_W << delta_x,
  delta_y,
  delta_theta;
  ROS_INFO_STREAM("Xi_W: \n" << Xi_W);

  Eigen::Matrix <float, 3, 1> Goal_pose;
  Goal_pose << delta_x,
  delta_y,
  delta_theta;
  ROS_INFO_STREAM("Goal_pose: \n" << Goal_pose);

  Eigen::MatrixXf phi_l_matrix, phi_r_matrix;
  phi_l_matrix = R_star_l * RotationMatrix * Xi_W * 1/r;
  phi_r_matrix = R_star_r * RotationMatrix * Xi_W * 1/r;
  phi_l = phi_l_matrix(0,0);
  phi_r = phi_r_matrix(0,0);


  ROS_INFO_STREAM("phi_l: \n" << phi_l);
  ROS_INFO_STREAM("phi_r: \n" << phi_r);

  float real_x = 0;
  float real_y = 0;
  float real_theta = 0;

  Eigen::Matrix <float, 3, 1> abs_pos_robot;
  abs_pos_robot << real_x,
  real_y,
  real_theta;

  ROS_INFO_STREAM(abs_pos_robot);

  float error_x = delta_x;
  float error_y = delta_y;
  float error_theta = delta_theta;


  Eigen::Matrix <float, 3, 1> difference_to_goal;
  difference_to_goal << error_x,
  error_y,
  error_theta;

  int count = 0;
  ROS_INFO_STREAM("\treal_x: " << abs_pos_robot(0,0) << "\treal_y: " << abs_pos_robot(1,0) << "\treal_theta: " << abs_pos_robot(2,0));
  
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    //Turn for 30 timestamps to the right
    if  (count >= 0 && count < 30){
      Xi_W << 0,
      0,
      deg2rad(2.166);


    }else if  (count >= 30 && count < 50){
      Xi_W << 0.35,
      0.75,
      0;
      

    }else  if(count >= 50 && count < 80){
      Xi_W << 0,
      0,
      deg2rad(-2.166);
    }
    else {
      Xi_W << 0,
      0,
      0;
    }

    //update values
    abs_pos_robot = abs_pos_robot + Xi_W * 1/hz;
    difference_to_goal = abs_pos_robot - Goal_pose;
    RotationMatrix << cos(abs_pos_robot(2,0)), sin(abs_pos_robot(2,0)), 0,
      -sin(abs_pos_robot(2,0)), cos(abs_pos_robot(2,0)),0,
      0,0,1; 

    phi_l_matrix = R_star_l * RotationMatrix * Xi_W * 1/r;
    phi_r_matrix = R_star_r * RotationMatrix * Xi_W * 1/r;


    ROS_INFO_STREAM("Position updated to (timestep " << count << "):");
    ROS_INFO_STREAM("\treal_x: " << abs_pos_robot(0,0) << "\treal_y: " << abs_pos_robot(1,0) << "\treal_theta: " << abs_pos_robot(2,0));
    ROS_INFO_STREAM("\terror_x: " << difference_to_goal(0,0) << "\terror_y: " << difference_to_goal(1,0) << "\terror_theta: " << difference_to_goal(2,0));
  
    ss << "Timestep " << count << ": phi_l = " << phi_l_matrix << "\tphi_r = " << phi_r_matrix;

    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    ROS_INFO("%s", msg.data.c_str());
  }

  return 0;
}   

