#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SpringNode : public rclcpp::Node {
public:
    SpringNode() : Node("Spring_Node") {
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&SpringNode::nav_callback, this, std::placeholders::_1));
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>("/yolo_bbox_raw", 10, std::bind(&SpringNode::yolo_callback, this, std::placeholders::_1));
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&SpringNode::img_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void yolo_callback(const std_msgs::msg::String::SharedPtr msg) {
        last_bbox_data = msg->data; // "label,x,y,w,h" 저장
    }

    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        draw_and_show(cv_ptr->image);
    }

    void draw_and_show(cv::Mat &img) {
        if (!last_bbox_data.empty()) {
            std::stringstream ss(last_bbox_data);
            std::string label, sx, sy, sw, sh;
            std::getline(ss, label, ','); std::getline(ss, sx, ','); std::getline(ss, sy, ',');
            std::getline(ss, sw, ','); std::getline(ss, sh, ',');
            try {
                int x = std::stoi(sx), y = std::stoi(sy), w = std::stoi(sw), h = std::stoi(sh);
                cv::Scalar color = (label == "ally") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                cv::rectangle(img, cv::Rect(x, y, w, h), color, 3);
                cv::putText(img, label, cv::Point(x, y-10), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
            } catch (...) {}
        }
        cv::imshow("Spring_Vision", img);
        cv::waitKey(1);
    }

    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out_msg = *msg;
        out_msg.linear.x *= 0.6; // 안개구간 서행
        cmd_pub_->publish(out_msg);
    }

    std::string last_bbox_data;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SpringNode>()); rclcpp::shutdown(); return 0; }
