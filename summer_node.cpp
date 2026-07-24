#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SummerNode : public rclcpp::Node {
public:
    SummerNode() : Node("Summer_Node") {
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&SummerNode::nav_callback, this, std::placeholders::_1));
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>("/yolo_bbox_raw", 10, std::bind(&SummerNode::yolo_callback, this, std::placeholders::_1));
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&SummerNode::img_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void yolo_callback(const std_msgs::msg::String::SharedPtr msg) {
        last_bbox_data = msg->data;
        if (last_bbox_data.find("red_light") != std::string::npos) g_traffic_signal_stop = true;
        else if (last_bbox_data.find("green_light") != std::string::npos) g_traffic_signal_stop = false;
        else if (last_bbox_data.find("supply_box") != std::string::npos && !is_supply_box_waiting) {
            is_supply_box_waiting = true; supply_box_start_time = this->now();
        }
    }

    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        // 화면에 박스 그리기
        if (!last_bbox_data.empty()) {
            std::stringstream ss(last_bbox_data);
            std::string l, sx, sy, sw, sh;
            std::getline(ss, l, ','); std::getline(ss, sx, ','); std::getline(ss, sy, ',');
            std::getline(ss, sw, ','); std::getline(ss, sh, ',');
            try {
                cv::rectangle(cv_ptr->image, cv::Rect(std::stoi(sx), std::stoi(sy), std::stoi(sw), std::stoi(sh)), cv::Scalar(255, 255, 0), 3);
            } catch (...) {}
        }
        cv::imshow("Summer_Vision", cv_ptr->image);
        cv::waitKey(1);
    }

    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out_msg = geometry_msgs::msg::Twist();
        if (g_traffic_signal_stop || (is_supply_box_waiting && (this->now() - supply_box_start_time).seconds() < 20.0)) {
            out_msg.linear.x = 0.0; out_msg.angular.z = 0.0;
        } else {
            is_supply_box_waiting = false; out_msg = *msg;
        }
        cmd_pub_->publish(out_msg);
    }

    std::string last_bbox_data;
    bool g_traffic_signal_stop = false, is_supply_box_waiting = false;
    rclcpp::Time supply_box_start_time;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SummerNode>()); rclcpp::shutdown(); return 0; }
