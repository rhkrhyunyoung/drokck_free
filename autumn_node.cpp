#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class AutumnNode : public rclcpp::Node {
public:
    AutumnNode() : Node("Autumn_Node") {
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&AutumnNode::nav_callback, this, std::placeholders::_1));
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>("/yolo_bbox_raw", 10, std::bind(&AutumnNode::yolo_callback, this, std::placeholders::_1));
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&AutumnNode::img_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void yolo_callback(const std_msgs::msg::String::SharedPtr msg) { last_bbox_data = msg->data; }

    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        if (!last_bbox_data.empty()) {
            // 박스 그리기 로직 (위와 동일)
            cv::putText(cv_ptr->image, "Autumn Marker Detected", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 165, 255), 2);
        }
        cv::imshow("Autumn_Vision", cv_ptr->image);
        cv::waitKey(1);
    }

    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_pub_->publish(*msg); }

    std::string last_bbox_data;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<AutumnNode>()); rclcpp::shutdown(); return 0; }
