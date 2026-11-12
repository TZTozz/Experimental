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
        }
    
    private:
        void info_callback(const custom_msgs::msg::Box::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Info marker: %d,\nPose: %d, %d \nDistance: %f m", msg->id, msg->cx, msg->cy, msg->d);
            ux_ = msg->cx;
            uy_ = msg->cy;
            d_ = msg->d;
            id_ = msg->id;
            
            // velocity calculation
            // Width: 640
             
            int cx = 640 / 2;
            int s_d = 166;
            float k_w = 0.01;
            float k_v = 0.02;
            float f = 381.3611602783203;
            float L = 0.2; //marker real size
            geometry_msgs::msg::Twist msg_velocities;

            float omega = - k_w * (ux_ - cx);
            float s = f * L / d_;
            float v = k_v * (s_d - s);


            // Goal reached feedback
            std_msgs::msg::Int32 msg_feedback;
            if ((d_) < 0.5){
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
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Velocity>());
    rclcpp::shutdown();
    return 0;
}