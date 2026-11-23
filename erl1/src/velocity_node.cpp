#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "custom_msgs/msg/box.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define threshold 2

using std::placeholders::_1;

class Velocity: public rclcpp::Node
{
    public:
        Velocity(): Node("velocity_node")
        {
            // Subscriber to marker_info
            this->declare_parameter<std::string>("marker_info_topic", "marker_info");       //name for declaring, default name
            std::string topic_name;
            this->get_parameter("marker_info_topic", topic_name);

            subscription_ = this->create_subscription<custom_msgs::msg::Box>(topic_name, 10, std::bind(&Velocity::info_callback, this, _1));

            // Publisher velocities
            this->declare_parameter<std::string>("velocity_topic", "cmd_vel");      
            std::string topic_pub_name;
            this->get_parameter("velocity_topic", topic_pub_name);

            publisherVelocities_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_pub_name, 10);

            // Publisher if goal is reached
            this->declare_parameter<std::string>("feedback_topic", "feedback_info");       
            std::string topic_pub_feedback_name;
            this->get_parameter("feedback_topic", topic_pub_feedback_name);
            
            publisherFeedback_ = this->create_publisher<std_msgs::msg::Int32>(topic_pub_feedback_name, 10);

            // Usefull parameters 
            this->declare_parameter<double>("marker_size", 0.2);
            this->declare_parameter<double>("focal_lenght", 381.3611602783203);

            L = this->get_parameter("marker_size").as_double();
            f = this->get_parameter("focal_lenght").as_double();
        }
    
    private:
        void info_callback(const custom_msgs::msg::Box::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Info marker: %d,\nCenter of the marker (x,y): %d px, %d px \nDistance: %f m", msg->id, msg->cx, msg->cy, msg->d);
            ux_ = msg->cx;
            uy_ = msg->cy;
            d_ = msg->d;
            id_ = msg->id;
            
            // velocity calculation
            // Width of the image: 640 px
             
            int cx = 640 / 2;       // Finds the center
            int s_d = 166;          // Desidered size of the marker in the image
            float k_w = 0.01;       // Proportional gain for angular velocity
            float k_v = 0.05;       // Proportional gain for linear velocity
            geometry_msgs::msg::Twist msg_velocities;

            float omega = - k_w * (ux_ - cx);   // Angular velocity
            float s = f * L / d_;               // Size of the marker in the image
            float v = k_v * (s_d - s);          // Linear velocity


            // Goal reached feedback
            std_msgs::msg::Int32 msg_feedback;
            if ((d_) < 0.5){        // If the distance is below 0.5m
                v = 0;
                omega = 0;
                msg_feedback.data = id_;
                RCLCPP_INFO(this->get_logger(), "Goal %d reached, px marker: %.0f", msg_feedback.data, s);
                publisherFeedback_->publish(msg_feedback);
            }

            // Publish velocities
            msg_velocities.linear.x = v;
            msg_velocities.angular.z = omega;
            publisherVelocities_->publish(msg_velocities);


        }

        rclcpp::Subscription<custom_msgs::msg::Box>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisherVelocities_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisherFeedback_;
        int ux_, uy_, id_;
        float d_;
        double L, f;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Velocity>());
    rclcpp::shutdown();
    return 0;
}