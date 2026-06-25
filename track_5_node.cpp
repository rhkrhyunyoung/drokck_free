#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <algorithm>

using namespace std::chrono_literals;

class Track5MissionManager : public rclcpp::Node {
public:
    Track5MissionManager() : Node("Track_5_Mission_Node") {
        // 구독자 설정
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&Track5MissionManager::nav_callback, this, std::placeholders::_1));
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_detected_object", 10, std::bind(&Track5MissionManager::yolo_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->current_yaw_rate_ = msg->angular_velocity.z;
            });

        // 발행자 설정
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "🚀 5구간 미션 매니저 가동 시작!");
    }

private:
    void yolo_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string label = msg->data;
        if (label == "STOP") g_traffic_signal_stop = true;
        else if (label == "GO") g_traffic_signal_stop = false;
    }

    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out_msg = geometry_msgs::msg::Twist();
        
        if (g_traffic_signal_stop) {
            out_msg.linear.x = 0.0; out_msg.angular.z = 0.0;
        } else {
            // 5구간은 특수 구간일 수 있으므로 필요시 여기서 속도 배율 조절 가능
            out_msg.linear.x = msg->linear.x;
            out_msg.angular.z = msg->angular.z;
        }
        cmd_pub_->publish(out_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    bool g_traffic_signal_stop = false;
    double current_yaw_rate_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Track5MissionManager>());
    rclcpp::shutdown();
    return 0;
}
