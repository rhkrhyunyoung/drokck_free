#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>

class SpringNode : public rclcpp::Node {
public:
    SpringNode() : Node("Spring_Node") {
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&SpringNode::nav_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->current_yaw_rate_ = msg->angular_velocity.z; });
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>("/yolo_detected_object", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "ally") RCLCPP_INFO(this->get_logger(), ">> FRIEND Identified");
            else if (msg->data == "enemy") RCLCPP_ERROR(this->get_logger(), ">> ENEMY Identified");
        });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
private:
    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out = *msg;
        double multiplier = 1.0;
        // 커브 저항 돌파 로직: 회전 명령 대비 실제 회전이 부족하면 토크 2.5배
        if (std::abs(msg->angular.z) > 0.3 && std::abs(msg->angular.z - current_yaw_rate_) > 0.2) {
            multiplier = 2.5;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Spring Curve Torque Boost!");
        }
        out.linear.x = msg->linear.x * 0.6 * multiplier; // 베이스 속도는 낮게, 토크는 높게
        out.angular.z = msg->angular.z * multiplier;
        cmd_pub_->publish(out);
    }
    double current_yaw_rate_ = 0.0;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SpringNode>()); rclcpp::shutdown(); return 0; }
