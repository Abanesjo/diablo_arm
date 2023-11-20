#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <iostream>
#include <string>
#include <time.h>
#include <sstream>
#include <Eigen/Dense>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"


namespace gazebo
{
class DiabloControlPlugin: public ModelPlugin
{
private:
  int JUMP_count;
  bool JUMPING;
  physics::LinkPtr base_link;
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;

  rclcpp::Node::SharedPtr nh;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_cmd;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_cmd, right_cmd, m0r_cmd, m0l_cmd, m1r_cmd, m1l_cmd;
  double LIM_, smooth_x_vel;
  double lin_x_cmd, ang_z_cmd, integral, Kp, Kd, Ki, vel_int, height_cmd, roll_cmd, JUMP_hstar;


  void Update()
    {
      ignition::math::Pose3d pose = (this->base_link)->WorldCoGPose();
      ignition::math::Quaternion orientation = pose.Rot();
      ignition::math::Vector3d lin_vel = (this->base_link)->RelativeLinearVel();
      ignition::math::Vector3d ang_vel = (this->base_link)->RelativeAngularVel();
      ignition::math::Vector3d euler_angles = orientation.Euler();

      smooth_x_vel = 0.95*smooth_x_vel + 0.05*(this->lin_x_cmd);
      double vel_error = lin_vel.X() - smooth_x_vel ;
      this->vel_int += 0.005 * vel_error;
      if((this->vel_int)>0.2){
        this->vel_int = 0.2;
      }if((this->vel_int)<-0.2){
        this->vel_int = -0.2;
      }
      double des_pitch = 0.04*smooth_x_vel -1.0*vel_error - 0.1*(this->vel_int);
      
      // double ang_vel_error = ang_vel.Z() - (this->ang_z_cmd) ;
      // double des_ang_z_rate = -1.2*ang_vel_error ;
      
      double pitch_error = euler_angles.Y() - des_pitch ;
      this->integral += 0.009 * pitch_error;
      if((this->integral)>LIM_){
        this->integral = LIM_;
      }if((this->integral)<-LIM_){
        this->integral = -LIM_;
      }
      double rate = ang_vel.Y();
      
      double real_Kp = this->Kp + 100.0*abs(euler_angles.Y()) ;
      double des_vel = real_Kp*pitch_error + (this->Ki)*(this->integral) + (this->Kd)*rate ;
      //des_vel *= (1.0 + 2.0*(this->ang_z_cmd));

      std_msgs::msg::Float64 r_msg, l_msg;
      r_msg.data = 0.5*des_vel + 4.0*(this->ang_z_cmd);
      l_msg.data = 0.5*des_vel - 4.0*(this->ang_z_cmd);
      left_cmd->publish(l_msg);
      right_cmd->publish(r_msg);

      if(this->JUMPING){
        this->JUMP_count++;
        this->height_cmd = 0.15 ;
        if(this->JUMP_count > 1200){
          this->height_cmd = 0.9;
        }
        if(this->JUMP_count > 1260){
          this->height_cmd = this->JUMP_hstar ;
          this->JUMP_count = 0;
          this->JUMPING = false;
        }
      }
      double height_l, height_r;
      height_r = -0.5*(this->roll_cmd);
      height_l = 0.5*(this->roll_cmd);

      //map height 0.5 to P0=0, P1=0
      double p0_r = -0.6*(this->height_cmd + height_r - 0.5) ;
      double p1_r = 1.05*(this->height_cmd + height_r - 0.5) ;
      double p0_l = -0.6*(this->height_cmd + height_l - 0.5) ;
      double p1_l = 1.05*(this->height_cmd + height_l - 0.5) ;		
      std_msgs::msg::Float64 msg_out;
      msg_out.data = p0_r;
      m0r_cmd->publish(msg_out);
      msg_out.data = p0_l;
      m0l_cmd->publish(msg_out);
      msg_out.data = p1_r;
      m1r_cmd->publish(msg_out);
      msg_out.data = p1_l;
      m1l_cmd->publish(msg_out);

    }

public:
  DiabloControlPlugin(){}

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      this->model = _parent;	
	  	smooth_x_vel = 0.0;		
	  	this->lin_x_cmd = 0.0;
	  	this->ang_z_cmd = 0.0;
	  	this->integral = 0.0;
	  	this->height_cmd = 0.5;
	  	this->roll_cmd = 0.0;
	  	LIM_ = 10.0;
	  	this->Kp = 250.0;
	  	this->Kd = 10.0;
	  	this->Ki = 70.0;
	  	this->vel_int = 0.0;
      this->JUMP_hstar = 0.5;
      this->JUMPING = false;
      this->JUMP_count = 0;

      std::string ns;
      if(_sdf->HasElement("namespace"))
        ns = _sdf->GetElement("namespace")->Get<std::string>();							
        
      this->base_link = this->model->GetLink(ns+"::Body");		
      
      std::string vel_topic = ns + "/vel_cmd" ;
      
      velocity_cmd = nh->create_subscription<geometry_msgs::msg::Twist>(
        vel_topic,
        1,
        [ = ](geometry_msgs::msg::Twist::SharedPtr msg)
        {
          vel_cmd_callback(msg);
        }
      );

      left_cmd = nh->create_publisher<std_msgs::msg::Float64>(ns+"/Motor2_L/vel_cmd", 1);
      right_cmd = nh->create_publisher<std_msgs::msg::Float64>(ns+"/Motor2_R/vel_cmd", 1); 			
      m0r_cmd = nh->create_publisher<std_msgs::msg::Float64>(ns+"/Motor0_R/pos_cmd", 1); 			
      m0l_cmd = nh->create_publisher<std_msgs::msg::Float64>(ns+"/Motor0_L/pos_cmd", 1); 					
      m1r_cmd = nh->create_publisher<std_msgs::msg::Float64>(ns+"/Motor1_R/pos_cmd", 1); 			
      m1l_cmd = nh->create_publisher<std_msgs::msg::Float64>(ns+"/Motor1_L/pos_cmd", 1);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&DiabloControlPlugin::Update, this));

    }

    void vel_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      if(msg->linear.y > 1.0 && !(this->JUMPING)){
        this->JUMPING = true;
        this->JUMP_count = 0;
        this->JUMP_hstar = this->height_cmd;
        this->roll_cmd = 0.0; 
        return;
      }

      this->lin_x_cmd = msg->linear.x;
      if(msg->linear.x > 0.3){
        this->lin_x_cmd = 0.3;
      }if(msg->linear.x < -0.3){
        this->lin_x_cmd = -0.3;
      }
      this->height_cmd = msg->linear.z + 0.5; //zero command is 0.5, middle height
      if((this->height_cmd)>1.0){
        this->height_cmd = 1.0;
      }if((this->height_cmd)<0.0){
        this->height_cmd = 0.0;
      }
      this->roll_cmd = msg->angular.x; //zero command is zero roll
      if((this->roll_cmd)>0.6){
        this->roll_cmd = 0.6;
      }if((this->roll_cmd)<-0.6){
        this->roll_cmd = -0.6;
      }
      
      
      this->ang_z_cmd = msg->angular.z;
      if(msg->angular.z > 0.3){
        this->ang_z_cmd = 0.3;
      }if(msg->angular.z < -0.3){
        this->ang_z_cmd = -0.3;
      }

      return;		
    }

 

};

GZ_REGISTER_MODEL_PLUGIN(DiabloControlPlugin)
}

