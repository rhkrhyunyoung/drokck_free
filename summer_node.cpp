#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

class SummerNode : public rclcpp::Node {
public:
    SummerNode() : Node("Summer_Node") {
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&SummerNode::nav_callback, this, std::placeholders::_1));
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>("/yolo_detected_object", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "STOP") stop_flag = true;
            else if (msg->data == "GO") stop_flag = false;
            else if (msg->data == "supply_box" && !box_done) { is_waiting = true; start_t = this->now(); }
        });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
private:
    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out = geometry_msgs::msg::Twist();
        if (stop_flag) { out.linear.x = 0; out.angular.z = 0; }
        else if (is_waiting) {
            double el = (this->now() - start_t).seconds();
            if (el < 20.0) { out.linear.x = 0; } else { is_waiting = false; box_done = true; }
        } else {
            out.linear.x = msg->linear.x * 1.5; // 험지 돌파용 기본 토크 업
            out.angular.z = msg->angular.z * 1.5;
        }
        cmd_pub_->publish(out);
    }
    bool stop_flag = false, is_waiting = false, box_done = false;
    rclcpp::Time start_t;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SummerNode>()); rclcpp::shutdown(); return 0; }
