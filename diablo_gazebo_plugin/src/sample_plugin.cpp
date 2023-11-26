#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include "std_msgs/msg/string.hpp"

namespace gazebo
{
    class SamplePlugin: public ModelPlugin
    {
        public: 
            SamplePlugin(){}
            
            void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
            {
                printf("Hello World!!!\n");

                model_ = model;
                world_ = model_->GetWorld();
                auto physicsEngine = world_->Physics();

                ros_node_ = gazebo_ros::Node::Get(sdf);
                RCLCPP_INFO(ros_node_->get_logger(), "loading sample plugin");

                talker_ = ros_node_->create_publisher<std_msgs::msg::String>("/talker", 10);
            }

        private:
            physics::ModelPtr model_;
            physics::WorldPtr world_;
            common::Time last_sim_time;
            common::Time last_update_time;
            double update_period_ms_;
            
            event::ConnectionPtr update_connection_;

            rclcpp::Node::SharedPtr ros_node_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr talker_;
            
            void Update()
            {
                RCLCPP_INFO(ros_node_->get_logger(), "hello");
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(SamplePlugin)
}