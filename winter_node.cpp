#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

class WinterNode : public rclcpp::Node {
public:
    WinterNode() : Node("Winter_Node") {
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&WinterNode::nav_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->yaw_rate = msg->angular_velocity.z; });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
private:
    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out = *msg;
        double mult = 1.5; // 기본 토크부터 빡세게
        if (std::abs(msg->linear.x) > 0.05 && std::abs(msg->angular.z - yaw_rate) > 0.3) {
            if (!slipping) { slipping = true; start_t = this->now(); }
            else {
                double dur = (this->now() - start_t).seconds();
                if (dur > 1.5) mult = 5.0; // 스티로폼 끼면 5배 토크로 씹어버림
            }
        } else { slipping = false; }
        out.linear.x *= mult;
        out.angular.z *= mult;
        cmd_pub_->publish(out);
    }
    double yaw_rate = 0.0; bool slipping = false; rclcpp::Time start_t;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<WinterNode>()); rclcpp::shutdown(); return 0; }
