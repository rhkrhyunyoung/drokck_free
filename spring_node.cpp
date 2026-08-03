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
        // 1. 카메라 구독자 설정
        sub_l.subscribe(this, "/camera_left/color/image_raw");
        sub_f.subscribe(this, "/camera_front/color/image_raw");
        sub_r.subscribe(this, "/camera_right/color/image_raw");

        // 2. 동기화 정책 설정 (가장 중요: 대기 시간과 오차 범위를 대폭 늘림)
        // 큐 사이즈를 100으로 늘리고, 시간 오차를 0.5초까지 허용
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100), sub_l, sub_f, sub_r);
        
        // 시간 오차 허용 한계치를 0.5초로 설정 (카메라가 버벅여도 무조건 합침)
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.5));
        sync_->registerCallback(std::bind(&SpringNode::process_all, this, 
                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // 3. 주행 및 YOLO 데이터 구독
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_bbox_raw", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("ally") != std::string::npos) RCLCPP_INFO(this->get_logger(), ">> ALLY FOUND!");
                else if (msg->data.find("enemy") != std::string::npos) RCLCPP_ERROR(this->get_logger(), ">> ENEMY FOUND!");
            });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->current_nav = *msg; });
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);

        RCLCPP_INFO(this->get_logger(), "=== [SPRING] AGX ORIN OPTIMIZED NODE STARTING ===");
    }

private:
    void process_all(const sensor_msgs::msg::Image::ConstSharedPtr& ml,
                     const sensor_msgs::msg::Image::ConstSharedPtr& mf,
                     const sensor_msgs::msg::Image::ConstSharedPtr& mr) 
    {
        try {
            // 1. 이미지 변환
            cv::Mat img_l = cv_bridge::toCvCopy(ml, "bgr8")->image;
            cv::Mat img_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
            cv::Mat img_r = cv_bridge::toCvCopy(mr, "bgr8")->image;

            // 2. 해상도 강제 통일 (중요!)
            // 아까 스크린샷에서 해상도가 제각각(720p, 480p)이었으므로 
            // 가로 640, 세로 480으로 3개 다 강제로 맞춥니다. (안 맞으면 합칠 때 터짐)
            cv::Size target_size(640, 480);
            cv::resize(img_l, img_l, target_size);
            cv::resize(img_f, img_f, target_size);
            cv::resize(img_r, img_r, target_size);

            // 3. 이미지 가로로 합치기
            cv::Mat combined;
            cv::hconcat(std::vector<cv::Mat>{img_l, img_f, img_r}, combined);

            // 4. 주행 제어 (봄 구간 60% 서행)
            auto out_msg = current_nav;
            out_msg.linear.x *= 0.6;
            cmd_pub_->publish(out_msg);

            // 5. UI용으로 다시 절반 리사이즈 (전송 속도 향상)
            cv::Mat ui_view;
            cv::resize(combined, ui_view, cv::Size(), 0.5, 0.5);

            // 6. UI 및 화면 출력
            auto msg = cv_bridge::CvImage(ml->header, "bgr8", ui_view).toImageMsg();
            ui_image_pub_->publish(*msg);

            cv::imshow("Integrated_3Cam_Vision", ui_view);
            cv::waitKey(1);

        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what());
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
