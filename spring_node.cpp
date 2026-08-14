#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SpringNode : public rclcpp::Node {
public:
    SpringNode() : Node("Spring_Node") {
        // 초기 시간 설정 (코어 덤프 방지)
        stuck_start_time = this->now();
        boost_start_time = this->now();
        cool_down_start_time = this->now();

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&SpringNode::process_front, this, std::placeholders::_1));
        
        // [중요] QoS 설정을 SensorDataQoS로 변경 (Best Effort 통신 허용)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/camera/imu", rclcpp::SensorDataQoS(), 
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->curr_ax = msg->linear_acceleration.x;
                this->curr_az = msg->linear_acceleration.z;
                this->imu_received = true;
            });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->current_nav = *msg; });
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);

        RCLCPP_INFO(this->get_logger(), "=== [SPRING] FINAL BOOST NODE STARTING ===");
    }

private:
    void process_front(const sensor_msgs::msg::Image::SharedPtr mf) {
        if (!imu_received) return; // IMU 데이터가 들어오기 전엔 실행 안 함

        auto now = this->now();
        auto out_msg = current_nav;

        // --- 로직 수정 포인트 ---
        
        // 1. 차체가 '어느 정도' 세워지고 있는 구간 (범위를 5.0으로 대폭 확장)
        // 뱅크에서 완전히 평지가 되기 전, 즉 끼이기 시작하는 각도에서도 작동하게 함
        bool is_transitioning = (std::abs(curr_ax) < 5.0); 

        // 2. 전진 저항 감지 (Z가 음수 방향으로 -1.0 이하)
        // 0.25m/s 속도에서 턱에 걸리면 순간적으로 -1.0 이하가 나옵니다.
        bool impact_z = (curr_az < -1.0); 

        // 3. 부스트 트리거 (네비 명령이 있고, 전환 구간이며, 충격이 감지될 때)
        if (std::abs(current_nav.linear.x) > 0.1 && is_transitioning && impact_z) {
            if (!is_boosting && !is_cool_down) {
                is_boosting = true;
                boost_start_time = now;
                RCLCPP_ERROR(this->get_logger(), "!!! HILL-CLIMB STUCK DETECTED - BOOST ON !!!");
            }
        }

        // --- 최대 속도 1.0m/s 부스트 동작 ---
        if (is_boosting) {
            double elapsed = (now - boost_start_time).seconds();
            if (elapsed < 2.0) { // 2초간 풀파워
                out_msg.linear.x = (current_nav.linear.x >= 0) ? 1.0 : -1.0;
                out_msg.angular.z = (current_nav.angular.z >= 0) ? 1.5 : -1.5;
            } else {
                is_boosting = false;
                is_cool_down = true; 
                cool_down_start_time = now;
            }
        }

        // 재작동 방지 쿨다운 (2초)
        if (is_cool_down) {
            if ((now - cool_down_start_time).seconds() > 2.0) is_cool_down = false;
        }

        cmd_pub_->publish(out_msg);

        // 디버깅 화면
        try {
            cv::Mat img = cv_bridge::toCvCopy(mf, "bgr8")->image;
            cv::resize(img, img, cv::Size(320, 240));
            
            if(is_boosting) {
                cv::rectangle(img, cv::Point(0,0), cv::Point(320,240), cv::Scalar(0,0,255), 6);
                cv::putText(img, "BOOSTING!!", cv::Point(100, 120), 1, 1.5, cv::Scalar(0,0,255), 2);
            }
            cv::imshow("Spring_Turbo", img);
            cv::waitKey(1);
        } catch (...) {}
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ui_image_pub_;

    geometry_msgs::msg::Twist current_nav;
    double curr_ax = 0.0, curr_az = 0.0;
    bool imu_received = false;
    bool is_boosting = false, is_cool_down = false;
    bool is_potential_stuck = false;
    rclcpp::Time stuck_start_time, boost_start_time, cool_down_start_time;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpringNode>());
    rclcpp::shutdown();
    return 0;
}
