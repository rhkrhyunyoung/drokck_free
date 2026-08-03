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
        // 1. 카메라 3개 구독 (정확한 리얼센스 토픽명)
        sub_l.subscribe(this, "/camera_left/color/image_raw");
        sub_f.subscribe(this, "/camera_front/color/image_raw");
        sub_r.subscribe(this, "/camera_right/color/image_raw");

        // 2. 시간 동기화 설정 (ApproximateTime)
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_l, sub_f, sub_r);
        sync_->registerCallback(std::bind(&SpringNode::process_all, this, 
                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // 3. 주행 및 YOLO 데이터 구독
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_bbox_raw", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("ally") != std::string::npos) RCLCPP_INFO(this->get_logger(), ">> ALLY FOUND!");
                else if (msg->data.find("enemy") != std::string::npos) RCLCPP_ERROR(this->get_logger(), ">> ENEMY FOUND!");
            });

        // 라인 트래킹 노드로부터 주행 값 받기
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->current_nav = *msg; });
        
        // 4. 퍼블리셔: 최종 제어 명령 및 UI용 통합 영상
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);

        RCLCPP_INFO(this->get_logger(), "=== [SPRING] 3-Cam UI & Mission Node Started ===");
    }

private:
    void process_all(const sensor_msgs::msg::Image::ConstSharedPtr& ml,
                     const sensor_msgs::msg::Image::ConstSharedPtr& mf,
                     const sensor_msgs::msg::Image::ConstSharedPtr& mr) 
    {
        // 성능 최적화: 2프레임당 1프레임만 처리 (랙 방지)
        static int frame_count = 0;
        if (++frame_count % 2 != 0) return;

        try {
            // 1. ROS 이미지를 OpenCV로 변환
            auto cv_l = cv_bridge::toCvCopy(ml, "bgr8")->image;
            auto cv_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
            auto cv_r = cv_bridge::toCvCopy(mr, "bgr8")->image;

            // 2. 가로로 합치기 (왼쪽 | 정면 | 오른쪽)
            cv::Mat combined;
            cv::hconcat(std::vector<cv::Mat>{cv_l, cv_f, cv_r}, combined);

            // 3. 주행 제어 로직 (봄: 안개 구간 60% 서행)
            auto out_msg = current_nav;
            out_msg.linear.x *= 0.6; 
            cmd_pub_->publish(out_msg);

            // 4. UI 전송용 최적화 (크기를 0.5배로 줄여서 전송 부하 감소)
            cv::Mat ui_view;
            cv::resize(combined, ui_view, cv::Size(), 0.5, 0.5);
            
            // 5. 통합 영상을 UI 토픽으로 발행
            auto msg = cv_bridge::CvImage(ml->header, "bgr8", ui_view).toImageMsg();
            ui_image_pub_->publish(*msg);

            // 로컬 모니터링용 (필요 없으면 주석 처리 가능)
            cv::imshow("Robot_Triple_Vision", ui_view);
            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_l, sub_f, sub_r;
    
    geometry_msgs::msg::Twist current_nav;
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
