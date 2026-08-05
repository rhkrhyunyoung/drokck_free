#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

class AutumnNode : public rclcpp::Node {
public:
    AutumnNode() : Node("Autumn_Node") {
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&AutumnNode::nav_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->yaw_rate = msg->angular_velocity.z; });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
private:
    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out = *msg;
        double boost = 1.0;
        // 긴 차체 커브 강제 돌파: 회전각이 큰데 안 돌면 3배 토크
        if (std::abs(msg->angular.z) > 0.4 && std::abs(msg->angular.z - yaw_rate) > 0.25) {
            boost = 3.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Autumn Curve Force Mode!");
        }
        out.linear.x = msg->linear.x * boost;
        out.angular.z = msg->angular.z * boost;
        cmd_pub_->publish(out);
    }
    double yaw_rate = 0.0;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<AutumnNode>()); rclcpp::shutdown(); return 0; }
