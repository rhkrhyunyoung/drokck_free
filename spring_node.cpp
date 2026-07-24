#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SpringNode : public rclcpp::Node {
public:
    SpringNode() : Node("Spring_Node") {
        // 1. 카메라 3개 구독 (ros2 topic list 결과와 정확히 일치시킴)
        sub_l.subscribe(this, "/camera_left/color/image_raw");
        sub_f.subscribe(this, "/camera_front/color/image_raw");
        sub_r.subscribe(this, "/camera_right/color/image_raw");

        // 2. 시간 동기화 설정
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_l, sub_f, sub_r);
        sync_->registerCallback(std::bind(&SpringNode::combined_callback, this, 
                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // 3. 주행 및 YOLO 데이터
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_bbox_raw", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("ally") != std::string::npos) RCLCPP_INFO(this->get_logger(), ">> ALLY!");
                else if (msg->data.find("enemy") != std::string::npos) RCLCPP_ERROR(this->get_logger(), ">> ENEMY!");
            });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->current_nav = *msg; });
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "=== [SPRING] 3-Cam Full Mission Mode ON ===");
    }

private:
    void combined_callback(const sensor_msgs::msg::Image::ConstSharedPtr& ml,
                           const sensor_msgs::msg::Image::ConstSharedPtr& mf,
                           const sensor_msgs::msg::Image::ConstSharedPtr& mr) 
    {
        auto cv_l = cv_bridge::toCvCopy(ml, "bgr8")->image;
        auto cv_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
        auto cv_r = cv_bridge::toCvCopy(mr, "bgr8")->image;

        // 가로로 합치기 (왼쪽 | 정면 | 오른쪽)
        cv::Mat combined;
        cv::hconcat(std::vector<cv::Mat>{cv_l, cv_f, cv_r}, combined);

        // 주행 명령: 봄 구간 서행 (60%)
        auto out_msg = current_nav;
        out_msg.linear.x *= 0.6;
        cmd_pub_->publish(out_msg);

        // 화면 띄우기
        cv::imshow("Robot_Triple_Vision", combined);
        cv::waitKey(1);
    }

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_l, sub_f, sub_r;
    
    geometry_msgs::msg::Twist current_nav;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpringNode>());
    rclcpp::shutdown();
    return 0;
}
