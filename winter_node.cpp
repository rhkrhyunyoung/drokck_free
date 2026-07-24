#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class WinterNode : public rclcpp::Node {
public:
    WinterNode() : Node("Winter_Node") {
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&WinterNode::nav_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->current_yaw_rate_ = msg->angular_velocity.z; });
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            if (this->is_slipping) cv::putText(cv_ptr->image, "!!! SLIP !!!", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 3);
            cv::imshow("Winter_Vision", cv_ptr->image);
            cv::waitKey(1);
        });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out_msg = geometry_msgs::msg::Twist();
        double raw_linear = msg->linear.x;
        double raw_angular = msg->angular.z;

        if (std::abs(raw_linear) > 0.05 && std::abs(raw_angular - current_yaw_rate_) > 0.3) {
            if (!is_slipping) { is_slipping = true; slip_start_time = this->now(); }
            else if ((this->now() - slip_start_time).seconds() > 1.5) torque_multiplier = 3.5;
        } else { is_slipping = false; torque_multiplier = 1.1; }

        out_msg.linear.x = raw_linear * torque_multiplier;
        out_msg.angular.z = std::clamp(raw_angular, -1.2, 1.2);
        cmd_pub_->publish(out_msg);
    }

    bool is_slipping = false;
    double current_yaw_rate_ = 0.0, torque_multiplier = 1.1;
    rclcpp::Time slip_start_time;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<WinterNode>()); rclcpp::shutdown(); return 0; }
