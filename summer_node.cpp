#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp> // IMU 메시지 추가
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SummerNode : public rclcpp::Node {
public:
    SummerNode() : Node("Summer_Node") {
        // 전방 카메라 구독
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, 
            std::bind(&SummerNode::process_front, this, std::placeholders::_1));

        // IMU 데이터 구독 추가
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->current_imu_linear_x = msg->linear_acceleration.x;
            });

        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_bbox_raw", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            std::string data = msg->data;
            auto status_msg = std_msgs::msg::String();

            if (data.find("red_light") != std::string::npos) {
                is_red_light = true;
                status_msg.data = "STOP";
                status_pub_->publish(status_msg);
            } 
            else if (data.find("green_light") != std::string::npos) {
                is_red_light = false;
                status_msg.data = "GO";
                status_pub_->publish(status_msg);
            }
            
            if (data.find("supply_box") != std::string::npos && !is_waiting) {
                is_waiting = true;
                wait_start_time = this->now();
            }
        });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { 
                this->current_nav = *msg; 
            });

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/light_status", 10);
    }

private:
    void process_front(const sensor_msgs::msg::Image::SharedPtr mf) {
        try {
            cv::Mat img_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
            cv::Size target_size(640, 480);
            cv::resize(img_f, img_f, target_size);

            auto out_msg = geometry_msgs::msg::Twist();

            // 1. 신호등 및 보급상자 제어 로직
            if (is_red_light) {
                out_msg.linear.x = 0.0; 
                out_msg.angular.z = 0.0;
            } 
            else if (is_waiting) {
                double elapsed = (this->now() - wait_start_time).seconds();
                if (elapsed < 20.0) { 
                    out_msg.linear.x = 0.0; 
                    out_msg.angular.z = 0.0; 
                } else { 
                    is_waiting = false; 
                    out_msg = current_nav; 
                }
            } 
            else {
                out_msg = current_nav;
            }
            
            // 2. IMU linear x가 0.1보다 작으면 토크(속도) 2배 증가 (정지 상태가 아닐 때만 적용)
            if (out_msg.linear.x != 0.0 && std::abs(current_imu_linear_x) < 0.1) {
                out_msg.linear.x *= 2.0;
            }

            cmd_pub_->publish(out_msg);

            // 시각화
            cv::Mat ui_view; 
            cv::resize(img_f, ui_view, cv::Size(), 0.5, 0.5);
            auto msg = cv_bridge::CvImage(mf->header, "bgr8", ui_view).toImageMsg();
            ui_image_pub_->publish(*msg);
            
            cv::imshow("Summer_Front_Vision", ui_view); 
            cv::waitKey(1);

        } catch (cv::Exception& e) { 
            RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what()); 
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // IMU 구독자 추가
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ui_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    geometry_msgs::msg::Twist current_nav;
    double current_imu_linear_x = 0.0; // IMU 가속도 저장 변수
    bool is_red_light = false;
    bool is_waiting = false;
    rclcpp::Time wait_start_time;
};

int main(int argc, char **argv) { 
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<SummerNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}
