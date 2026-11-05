#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "custom_msg/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

#define threshold 2

using std::placeholders::_1;

class Visual: public rclcpp::Node
{
    public:
        Visual(): Node("visual_node")
        {
            // Configurable topic
            this->declare_parameter<std::string>("marker_info_topic", "marker_info");       //name for declaring, defaiult name
            std::string topic_name;
            this->get_parameter("marker_info_topic", topic_name);

            
            subscription_ = this->create_subscription<>(topic_name, 10, std::bind(&Visual::info_callback, this, _1));
            
            publisherVelocities_ = this->create_publisher<geometry::msg::Twist>("velocities", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Distance::timer_callback, this));
            msg_stop.data = false;
        }
    
    private:
        void info_callback(const custom_msg::msg::Marker::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Info marker: %d,\nPose: %d, %d \nSize: %d px", msg->id, msg->x, msg->y, msg->size);
            x1_ = msg->x;
            y1_ = msg->y;
            size_ = msg->size;
            id_ = msg->id;
            
            // velocity calculation
            // Width: 640
             
            int cx = 640 / 2;
            int s_d = 166;
            float k_w = 0.1;
            float k_v = 0.1;
            float f = 381.3611602783203;
            geometry::msg::Twist msg_velocities;

            float omega = - k_w * (x1_ - cx);
            float s = f * L / d;
            float v = k_v * (s_d - s);




        }


        void timer_callback()
        {
            //Compute the distance
            float xDistance_ = fabs(x1_ - x2_);
            float yDistance_ = fabs(y1_ - y2_);

            
            msg_distance.data = sqrt(pow(xDistance_, 2) + pow(yDistance_, 2));

            RCLCPP_INFO(this->get_logger(), "The distance is: %f", msg_distance.data);
            publisherDistance_->publish(msg_distance);

            if (msg_distance.data < threshold || x1_ > 10.0 || x1_ < 1.0 || y1_ > 10.0 || y1_ < 1.0 || x2_ > 10.0 || x2_ < 1.0 || y2_ > 10.0 || y2_ < 1.0)
            {
                msg_stop.data = true;
                publisherStop_->publish(msg_stop);
                RCLCPP_INFO(this->get_logger(), "Stopping the turtles");
                alreadyStopped = true;
            }
            else
            {
                if (alreadyStopped) 
                {
                    msg_stop.data = false;
                    publisherStop_->publish(msg_stop);
                    RCLCPP_INFO(this->get_logger(), "The turtles can move");
                    alreadyStopped = false;
                }
            }

        }
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisherDistance_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisherStop_;
        rclcpp::TimerBase::SharedPtr timer_;
        std_msgs::msg::Float32 msg_distance;
        std_msgs::msg::Bool msg_stop;
        int x_, y_, size_, id_;
        bool alreadyStopped = false;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Distance>());
    rclcpp::shutdown();
    return 0;
}