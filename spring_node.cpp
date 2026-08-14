#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <algorithm> // std::clamp 사용

using namespace std::chrono_literals;

class SpringNode : public rclcpp::Node {
public:
    SpringNode() : Node("Spring_Node") {
        boost_start_time = this->now();
        cool_down_start_time = this->now();

        // 1. IMU 구독 (QoS 설정 필수)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/camera/imu", rclcpp::SensorDataQoS(), 
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->curr_ax = msg->linear_acceleration.x;
                this->curr_az = msg->linear_acceleration.z;
                this->imu_received = true;
            });

        // 2. 네비게이션 명령 구독
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { 
                this->current_nav = *msg; 
                this->run_control_logic(); 
            });
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "=== [SPRING] MULTIPLIER BOOST NODE STARTING ===");
    }

private:
    void run_control_logic() {
        if (!imu_received) return;

        auto now = this->now();
        auto out_msg = current_nav;

        // --- 마의 구간 감지 조건 ---
        // 1. 차체 세워짐 (|X| < 1.5)
        bool is_uprighting = (std::abs(curr_ax) < 1.5);
        // 2. 전진 충격/걸림 (Z < -1.0)
        bool impact_z = (curr_az < -1.0);

        if (std::abs(current_nav.linear.x) > 0.1 && is_uprighting && impact_z) {
            if (!is_boosting && !is_cool_down) {
                is_boosting = true;
                boost_start_time = now;
                RCLCPP_ERROR(this->get_logger(), ">> [BOOST] HILL DETECTED! Speed x4 Boosting...");
            }
        }

        // --- 배율 기반 부스트 동작 ---
        if (is_boosting) {
            double elapsed = (now - boost_start_time).seconds();
            if (elapsed < 2.5) { 
                // 1. 전진 속도: 기존 명령의 4배 증폭 (최대 1.0m/s 제한)
                double boosted_linear = current_nav.linear.x * 4.0;
                out_msg.linear.x = std::clamp(boosted_linear, -1.0, 1.0);

                // 2. 회전 속도: 기존 명령의 2배 증폭 (회전력 확보)
                out_msg.angular.z = current_nav.angular.z * 2.0;

                RCLCPP_INFO(this->get_logger(), ">> BOOSTING: %.2f -> %.2f m/s", 
                            current_nav.linear.x, out_msg.linear.x);
            } else {
                is_boosting = false;
                is_cool_down = true; 
                cool_down_start_time = now;
                RCLCPP_INFO(this->get_logger(), ">> [NORMAL] Boost Finished.");
            }
        }

        // 쿨다운 (3초)
        if (is_cool_down) {
            if ((now - cool_down_start_time).seconds() > 3.0) is_cool_down = false;
        }

        cmd_pub_->publish(out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    geometry_msgs::msg::Twist current_nav;
    double curr_ax = 0.0, curr_az = 0.0;
    bool imu_received = false;
    bool is_boosting = false, is_cool_down = false;
    rclcpp::Time boost_start_time, cool_down_start_time;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpringNode>());
    rclcpp::shutdown();
    return 0;
}
