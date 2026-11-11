#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp> // IMU 메시지 추가
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SpringNode : public rclcpp::Node {
public:
    SpringNode() : Node("Spring_Node") {
        // 1. 전방 카메라 토픽만 구독하도록 변경
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, 
            std::bind(&SpringNode::process_front, this, std::placeholders::_1));

        // 2. IMU 데이터 구독 추가
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->current_imu_linear_x = msg->linear_acceleration.x;
            });

        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_bbox_raw", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("ally") != std::string::npos) RCLCPP_INFO(this->get_logger(), ">> ALLY FOUND!");
                else if (msg->data.find("enemy") != std::string::npos) RCLCPP_ERROR(this->get_logger(), ">> ENEMY FOUND!");
            });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->current_nav = *msg; });
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);

        RCLCPP_INFO(this->get_logger(), "=== [SPRING] SINGLE CAM & IMU BOOST NODE STARTING ===");
    }

private:
    void process_front(const sensor_msgs::msg::Image::SharedPtr mf) 
    {
        try {
            cv::Mat img_f = cv_bridge::toCvCopy(mf, "bgr8")->image;

            cv::Size target_size(640, 480);
            cv::resize(img_f, img_f, target_size);

            // 주행 제어 로직 (기존 봄 구간 60% 서행 유지)
            auto out_msg = current_nav;
            out_msg.linear.x *= 0.6;

            // 2. IMU 데이터에서 linear x(가속도)가 0.1 이하이면 토크(속도 명령) 2배 증폭
            if (std::abs(current_imu_linear_x) <= 0.1) {
                out_msg.linear.x *= 2.0;
            }

            cmd_pub_->publish(out_msg);

            // UI용 리사이즈
            cv::Mat ui_view;
            cv::resize(img_f, ui_view, cv::Size(), 0.5, 0.5);

            auto msg = cv_bridge::CvImage(mf->header, "bgr8", ui_view).toImageMsg();
            ui_image_pub_->publish(*msg);

            cv::imshow("Front_Cam_Vision", ui_view);
            cv::waitKey(1);

        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what());
        }
    }

    // 변수 선언부 수정
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    geometry_msgs::msg::Twist current_nav;
    double current_imu_linear_x = 0.0; // IMU 값 저장 변수

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ui_image_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpringNode>());
    rclcpp::shutdown();
    return 0;
}
