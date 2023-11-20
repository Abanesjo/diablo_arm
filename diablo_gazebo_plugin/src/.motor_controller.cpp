#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <std_msgs/msg/float64.hpp>

#include <iostream>
#include <string>
#include <time.h>
#include <sstream>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"

namespace gazebo
{
class GenericMotorPlugin : public ModelPlugin
{
private:
    int mode;
    double cmd, integral, Kp, Kd, Ki;
    physics::JointPtr motor_joint;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_feedback;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_cmd, velocity_cmd;
    double LIM_;


    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->model = _parent;			
		this->mode = 0; //default mode is position
		this->cmd = 0.0;
		this->integral = 0.0;
		this->Kp = 0.5;
		this->Kd = 0.05;
		this->Ki = 0.1;
		LIM_ = 0.5;

		std::string name;
		std::string ns;
		if(_sdf->HasElement("namespace"))
			ns = _sdf->GetElement("namespace")->Get<std::string>();							
		if(_sdf->HasElement("joint_name"))
			name = _sdf->GetElement("joint_name")->Get<std::string>();		
		if(_sdf->HasElement("Kp"))
			this->Kp = _sdf->GetElement("Kp")->Get<double>();				
		if(_sdf->HasElement("Kd"))
			this->Kd = _sdf->GetElement("Kd")->Get<double>();				
		if(_sdf->HasElement("Ki"))
			this->Ki = _sdf->GetElement("Ki")->Get<double>();				
			
		this->motor_joint = this->model->GetJoint(ns+"::"+name);		
		
		std::string pos_topic = ns + "/" + name + "/pos_cmd" ;
		std::string vel_topic = ns + "/" + name + "/vel_cmd" ;

        position_cmd = nh->create_subscription<std_msgs::msg::Float64>(
            pos_topic,
            10,
            [=](std_msgs::msg::Float64::SharedPtr msg)
            {
                pos_cmd_callback(msg);
            }
        );
        velocity_cmd = nh->create_subscription<std_msgs::msg::Float64>(
            vel_topic,
            10,
            [=](std_msgs::msg::Float64::SharedPtr msg)
            {
                vel_cmd_callback(msg);
            }
        );

        torque_feedback = nh->create_publisher<std_msgs::msg::Float64>(ns+"/"+name+"/applied_torque", 1);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GenericMotorPlugin::Update, this));
    }

public:
    GenericMotorPlugin(){}

    void Update()
    {
        double torque = 0.0;
        double angle = this->motor_joint->Position(0);
        double rate = this->motor_joint->GetVelocity(0);

        if(mode==0){
            this->integral += 0.005 * (angle - this->cmd) ; 
            if(this->integral > LIM_){
                this->integral = LIM_;
            }
            if(this->integral < -LIM_){
                this->integral = -LIM_;
            }
            torque = -(this->Kp)*(angle - this->cmd) - 2.0*rate - 0.3*this->integral;			
            this->motor_joint->SetForce(0,torque);
            std_msgs::msg::Float64 feedback_msg;
            feedback_msg.data = torque;
            torque_feedback->publish(feedback_msg);
            return;
        }
        if(mode==1){
            this->integral += 0.005 * (rate - this->cmd) ; 
            if(this->integral > LIM_){
                this->integral = LIM_;
            }
            if(this->integral < -LIM_){
                this->integral = -LIM_;
            }
            torque = -(this->Kd)*(rate - this->cmd) - (this->Ki)*this->integral;
            this->motor_joint->SetForce(0,torque);
            std_msgs::msg::Float64 feedback_msg;
            feedback_msg.data = torque;
            torque_feedback->publish(feedback_msg);
            return;
        }
    }

    void pos_cmd_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {  
        if(this->mode != 0){
			this->integral = 0.0;
		}
		this->mode = 0;
		this->cmd = msg->data;
		return;	
    }

    void vel_cmd_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if(this->mode != 1){
			this->integral = 0.0;
		}
		this->mode = 1;
		this->cmd = msg->data;
		return;

    }
};
GZ_REGISTER_MODEL_PLUGIN(GenericMotorPlugin)
}