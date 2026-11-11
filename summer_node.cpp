// DROK_SUMMER_NODE_ONLY_V5_4_CAMERA_AFTER_RETURN
// Replace ONLY: ~/drokck_free/src/car/summer_node.cpp
// Runtime roots: ~/DROK_ARM_IK + ~/drokck_free
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <functional>
#include <sstream>
#include <string>
#include <vector>
#include <thread>

// DROK SummerNode - NUC one-command ARM supply-box supervisor
//
// Exact mission order:
//   1) Summer starts ARM feedback/TF/AutoGrasp and keeps chassis stopped.
//   2) AutoGrasp automatically moves J1~J6 from current q to captured START HOME.
//      Legacy manual-J1 preposition gate is disabled by Summer at launch.
//      v4.8 policy: startup gripper OPEN command is NOT sent.
//   3) Only after READY_FOR_DETECTION, Summer starts LOCAL D405 + supply YOLO.
//   4) After D405 topics are stable, open one 5.0 s stationary detection window.
//      The first valid supply_box class+XYZ observation inside the window latches GRASP.
//      AutoGrasp still performs its own 5-sample stable-target filtering before motion.
//   5-A) supply_box confirmed inside that window:
//        keep D405+YOLO alive for Stage1/2/3 re-measurements -> grasp/hold ->
//        return carrying HOME -> THEN stop D405+YOLO -> drive resumes.
//   5-B) no supply_box in that window:
//        stop D405+YOLO -> permanently disable late ARM triggers -> drive resumes.
//   6) Existing /cmd_vel_nav -> /cmd_vel path is passed through unchanged after release.
//
// One ROS node command is sufficient: ros2 run drokck summer_node.
// No separate camera/YOLO/ARM bridge/AutoGrasp terminal is required.
//
// NOTE: 20 cm pre-grasp / second D405 measurement / final constrained IK
// is implemented in the paired DROK_ARM_IK/tools/drok_auto_grasp_prototype1.py.

class SummerNode : public rclcpp::Node {
public:
    SummerNode() : Node("Summer_Node") {
        // =====================================================================
        // ARM / D405 USER CONFIGURATION
        // =====================================================================
        const char *home_env = std::getenv("HOME");
        const std::string home =
            (home_env != nullptr) ? std::string(home_env) : std::string("/home/kudos");

        const char *arm_ws_env = std::getenv("DROK_ARM_WS");
        const std::string default_arm_ws =
            (arm_ws_env != nullptr && arm_ws_env[0] != '\0')
                ? std::string(arm_ws_env)
                : (home + "/DROK_ARM_IK");

        arm_ws_root_ = this->declare_parameter<std::string>(
            "arm_ws_root", default_arm_ws);

        // v5.2 exact NUC layout:
        //   ~/DROK_ARM_IK
        //   ~/drokck_free
        // Summer itself launches the supporting processes from these roots.
        const std::string expected_arm_auto =
            arm_ws_root_ + "/tools/drok_auto_grasp_prototype1.py";
        const std::string expected_arm_core =
            arm_ws_root_ + "/tools/interactive_box_ik_grasp_v11.py";
        const std::string expected_arm_bridge =
            arm_ws_root_ + "/tools/run_real.sh";

        if (::access(expected_arm_auto.c_str(), R_OK) != 0 ||
            ::access(expected_arm_core.c_str(), R_OK) != 0 ||
            ::access(expected_arm_bridge.c_str(), R_OK) != 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[STARTUP BLOCK] Required ~/DROK_ARM_IK tools are missing. "
                "Expected AutoGrasp/core/run_real under %s/tools",
                arm_ws_root_.c_str());
            arm_startup_blocked_ = true;
        }


        // =====================================================================
        // LAPTOP D405 INTEGRATION TEST MODE
        // =====================================================================
        // This test-only SummerNode can start the locally connected D405 and
        // the copied Jetson YOLO package by itself.  Production NUC can disable
        // this and keep receiving the exact same YOLO topics over ROS2.
        test_root_ = this->declare_parameter<std::string>(
            "test_root", home + "/drokck_free");
        local_perception_enabled_ = this->declare_parameter<bool>(
            "local_perception_enabled", true);
        local_perception_ws_root_ = this->declare_parameter<std::string>(
            "local_perception_ws_root", test_root_);
        arm_auto_runner_path_ = this->declare_parameter<std::string>(
            "arm_auto_runner_path",
            arm_ws_root_ + "/tools/run_drok_auto_grasp_prototype1.sh");
        // v5.2 compatibility parameter only; AutoGrasp is launched directly
        // from drok_auto_grasp_prototype1.py inside start_auto_grasp_if_ready().

        local_camera_color_topic_ = this->declare_parameter<std::string>(
            "local_camera_color_topic", "/arm_d405/camera/color/image_raw");
        local_camera_depth_topic_ = this->declare_parameter<std::string>(
            "local_camera_depth_topic", "/arm_d405/camera/aligned_depth_to_color/image_raw");
        local_camera_info_topic_ = this->declare_parameter<std::string>(
            "local_camera_info_topic", "/arm_d405/camera/color/camera_info");

        local_perception_ready_timeout_sec_ = this->declare_parameter<double>(
            "local_perception_ready_timeout_sec", 20.0);
        local_perception_stable_sec_ = this->declare_parameter<double>(
            "local_perception_stable_sec", 0.75);

        // Arm D405 used in the verified laptop/NUC tests.
        // Override with --ros-args -p d405_serial:=... only if the physical camera changes.
        d405_serial_ = this->declare_parameter<std::string>(
            "d405_serial", "260322271364");

        // D405 is mounted on the moving GRIPPER_BASE.
        // SolidWorks nominal transform, all axes:
        //   +X forward, +Y left, +Z up
        //
        // GRIPPER_BASE -> camera_link
        //   x = -40.90 mm (behind)
        //   y =   0.00 mm
        //   z = +61.46 mm (above)
        //
        // q.py / Jetson must publish /yolo_object_xyz_supply in the same mechanical
        // camera_link convention: X forward, Y left, Z up.
        arm_tf_x_ = this->declare_parameter<double>("arm_tf_x", -0.04090);
        arm_tf_y_ = this->declare_parameter<double>("arm_tf_y",  0.00000);
        arm_tf_z_ = this->declare_parameter<double>("arm_tf_z", +0.06146);
        arm_tf_roll_ = this->declare_parameter<double>("arm_tf_roll", 0.0);
        arm_tf_pitch_ = this->declare_parameter<double>("arm_tf_pitch", 0.0);
        arm_tf_yaw_ = this->declare_parameter<double>("arm_tf_yaw", 0.0);

        // IMPORTANT: this is now GRIPPER_BASE -> camera_link, not
        // ARM_BASE_LINK -> camera_link. The ARM_BASE_LINK -> GRIPPER_BASE part
        // comes from robot_state_publisher and the live /joint_states chain.
        arm_tf_parent_frame_ = this->declare_parameter<std::string>(
            "arm_tf_parent_frame", "GRIPPER_BASE");
        arm_tf_child_frame_ = this->declare_parameter<std::string>(
            "arm_tf_child_frame", "camera_link");

        // The real-arm bridge publishes /joint_states, but the moving camera
        // needs the full ARM_BASE_LINK -> ... -> GRIPPER_BASE TF chain.
        // Summer can start robot_state_publisher from the same arm URDF so no
        // chassis/navigation file needs to be changed. Disable only when an
        // external robot_state_publisher already publishes this exact arm.
        arm_start_robot_state_publisher_ = this->declare_parameter<bool>(
            "arm_start_robot_state_publisher", true);
        const char *arm_urdf_env = std::getenv("DROK_ARM_URDF_PATH");
        const std::string default_arm_urdf =
            (arm_urdf_env != nullptr && arm_urdf_env[0] != '\0')
                ? std::string(arm_urdf_env)
                : (arm_ws_root_ + "/src/drok_arm_kinematics/config/drok_arm_kinematics_only.urdf");

        arm_urdf_path_ = this->declare_parameter<std::string>(
            "arm_urdf_path", default_arm_urdf);

        // After a valid box is confirmed, keep the chassis stopped briefly
        // before publishing the one-shot ARM enable.
        arm_chassis_settle_sec_ = this->declare_parameter<double>(
            "arm_chassis_settle_sec", 0.5);

        // Summer starts the real bridge first. AutoGrasp starts only after
        // real /joint_states are actually seen.
        arm_bridge_ready_timeout_sec_ = this->declare_parameter<double>(
            "arm_bridge_ready_timeout_sec", 15.0);
        arm_bridge_stable_sec_ = this->declare_parameter<double>(
            "arm_bridge_stable_sec", 0.5);
        arm_joint_state_stale_timeout_sec_ = this->declare_parameter<double>(
            "arm_joint_state_stale_timeout_sec", 1.0);

        arm_feedback_topics_ = this->declare_parameter<std::vector<std::string>>(
            "arm_feedback_topics",
            std::vector<std::string>{
                "/motor_angles/can10_motor_0x141",  // JOINT1
                "/motor_angles/can10_motor_0x142",  // JOINT2
                "/motor_angles/can10_motor_0x144",  // JOINT3
                "/motor_angles/can11_motor_0x141",  // JOINT4
                "/motor_angles/can11_motor_0x142",  // JOINT5
                "/motor_angles/can11_motor_0x143"   // JOINT6
            });

        if (arm_feedback_topics_.size() != 6) {
            RCLCPP_WARN(
                this->get_logger(),
                "[ARM] arm_feedback_topics must contain 6 topics. Restoring verified defaults.");
            arm_feedback_topics_ = {
                "/motor_angles/can10_motor_0x141",
                "/motor_angles/can10_motor_0x142",
                "/motor_angles/can10_motor_0x144",
                "/motor_angles/can11_motor_0x141",
                "/motor_angles/can11_motor_0x142",
                "/motor_angles/can11_motor_0x143"
            };
        }

        // =====================================================================
        // BOX SEARCH PARAMETERS - USER TUNABLE
        // =====================================================================
        // 1) AutoGrasp first moves J1~J6 to exact START HOME.
        //    Startup gripper position is untouched (v4.8).
        // 2) READY_FOR_DETECTION starts LOCAL D405 + supply YOLO.
        // 3) After perception is stable, exactly one stationary detection
        //    window is opened for box_search_timeout_sec.
        // 4) No box -> D405/YOLO stop and autonomous driving is released.
        //    Late supply-box triggers are intentionally disabled.
        // 5) Box -> perception stays alive through grasp refinements and is
        //    stopped only after GRASP_SECURED.
        box_search_timeout_sec_ = this->declare_parameter<double>(
            "box_search_timeout_sec", 5.0);

        // Distance is measured from camera_link to the YOLO 3D point.
        // Change these from the terminal without rebuilding.
        box_min_camera_distance_m_ = this->declare_parameter<double>(
            "box_min_camera_distance_m", 0.20);
        box_max_camera_distance_m_ = this->declare_parameter<double>(
            "box_max_camera_distance_m", 1.00);

        // Summer is only the 5-second "box present?" gate.
        // One valid class+XYZ observation inside the window latches the ARM mission.
        // AutoGrasp independently requires 5 stable XYZ samples before moving.
        box_detection_required_count_ = this->declare_parameter<int>(
            "box_detection_required_count", 1);
        class_xyz_association_max_sec_ = this->declare_parameter<double>(
            "class_xyz_association_max_sec", 0.25);
        box_detection_max_gap_sec_ = this->declare_parameter<double>(
            "box_detection_max_gap_sec", 0.40);

        if (box_search_timeout_sec_ < 0.1) {
            RCLCPP_WARN(
                this->get_logger(),
                "[BOX] box_search_timeout_sec too small. Forced to 0.1 sec.");
            box_search_timeout_sec_ = 0.1;
        }
        if (box_min_camera_distance_m_ < 0.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "[BOX] box_min_camera_distance_m < 0. Forced to 0.0 m.");
            box_min_camera_distance_m_ = 0.0;
        }
        if (box_max_camera_distance_m_ <= box_min_camera_distance_m_) {
            RCLCPP_WARN(
                this->get_logger(),
                "[BOX] max distance must be > min distance. Forced to min + 0.10 m.");
            box_max_camera_distance_m_ = box_min_camera_distance_m_ + 0.10;
        }
        if (box_detection_required_count_ < 1) {
            RCLCPP_WARN(
                this->get_logger(),
                "[BOX] box_detection_required_count < 1. Forced to 1.");
            box_detection_required_count_ = 1;
        }

        // =====================================================================
        // Original Summer camera view - retained, disabled by default
        // =====================================================================
        enable_camera_view_ = this->declare_parameter<bool>(
            "enable_camera_view", false);
        camera_image_topic_ = this->declare_parameter<std::string>(
            "camera_image_topic", "/camera/camera/color/image_raw");

        // =====================================================================
        // SUBSCRIPTIONS
        // =====================================================================

        // Local D405 readiness watchers use sensor-data QoS so they are
        // compatible with normal RealSense image publishers.  The arm is not
        // allowed to move until color + aligned depth + CameraInfo are alive.
        if (local_perception_enabled_) {
            local_color_ready_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                local_camera_color_topic_,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Image::SharedPtr) {
                    local_camera_color_seen_ = true;
                });

            local_depth_ready_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                local_camera_depth_topic_,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Image::SharedPtr) {
                    local_camera_depth_seen_ = true;
                });

            local_info_ready_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                local_camera_info_topic_,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr) {
                    local_camera_info_seen_ = true;
                });
        }

        // Existing Summer traffic-light logic is intentionally preserved.
        yolo_bbox_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_detected_object", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                const std::string data = msg->data;
                auto status_msg = std_msgs::msg::String();

                if (data.find("red_light") != std::string::npos || data.find("STOP") != std::string::npos) {
                    is_red_light_ = true;
                    status_msg.data = "STOP";
                    status_pub_->publish(status_msg);
                }
                else if (data.find("green_light") != std::string::npos || data.find("GO") != std::string::npos) {
                    is_red_light_ = false;
                    status_msg.data = "GO";
                    status_pub_->publish(status_msg);
                }
            });

        // Save only the most recent supply_box class time. XYZ is accepted only
        // when class and XYZ are temporally associated.
        yolo_object_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_detected_object_supply", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (!arm_ready_for_detection_ || arm_mission_completed_) {
                    return;
                }

                if (msg->data.find("supply_box") != std::string::npos) {
                    last_supply_box_class_time_ = this->now();
                    have_recent_supply_box_class_ = true;
                }
            });

        // Jetson D405 YOLO XYZ. Expected frame_id=camera_link and
        // vector=(forward, left, up) in metres.
        yolo_xyz_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/yolo_object_xyz_supply", 10,
            std::bind(
                &SummerNode::process_supply_box_xyz,
                this,
                std::placeholders::_1));

        // Existing autonomous navigation command input - unchanged.
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                current_nav_ = *msg;
            });

        // AutoGrasp publishes DONE=True only AFTER the object is secured and
        // the arm has physically completed its LOW RETURN HOME motion.
        //
        // v5.4 exact requested order:
        //   grasp -> hold object -> RETURN HOME -> D405/YOLO OFF -> drive resume.
        //
        // Keep is_waiting_=true while stopping perception so the chassis remains
        // STOPPED until both local perception child processes are shut down.
        arm_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drok_arm_auto/done", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && is_waiting_ && arm_started_) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[ARM] RETURN HOME COMPLETE WITH OBJECT -> now stopping LOCAL_YOLO + LOCAL_D405");

                    if (local_perception_enabled_ &&
                        !local_perception_intentionally_stopped_)
                    {
                        local_perception_intentionally_stopped_ = true;
                        local_perception_ready_ = false;
                        local_camera_color_seen_ = false;
                        local_camera_depth_seen_ = false;
                        local_camera_info_seen_ = false;

                        // Intentionally blocking here (normally <= ~1.2 s total):
                        // is_waiting_ stays true, therefore /cmd_vel remains STOP
                        // until both arm-camera processes have been asked to exit.
                        stop_child_process_group("LOCAL_YOLO", local_yolo_pid_);
                        stop_child_process_group("LOCAL_D405", local_realsense_pid_);
                    }

                    arm_mission_completed_ = true;
                    arm_started_ = false;
                    is_waiting_ = false;
                    arm_ready_for_detection_ = false;
                    initial_search_active_ = false;

                    RCLCPP_WARN(
                        this->get_logger(),
                        "[MISSION] OBJECT HELD AT RETURN HOME -> D405/YOLO OFF -> chassis autonomous drive RESUME");
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[ARM] Further supply_box grasp triggers are ignored until Summer restarts.");
                }
            });

        // AutoGrasp startup handshake. Current ARM code moves J1~J6 to exact HOME and
        // preserves the operator-set gripper opening before READY_FOR_DETECTION.
        arm_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/drok_arm_auto/status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                // AutoGrasp keeps the legacy status text for compatibility,
                // but v5.4 intentionally DOES NOT stop perception here.
                // Stage3 has finished and the gripper is closed; keep the arm D405
                // and supply YOLO alive while carrying the object back HOME.
                if (msg->data == "GRASP_SECURED_STOP_D405_PERCEPTION") {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[ARM] BOX SECURED -> KEEP LOCAL_D405 + LOCAL_YOLO ON during RETURN HOME");
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[ARM] Perception shutdown is deferred until /drok_arm_auto/done=True (RETURN HOME complete)");
                    return;
                }

                if (msg->data == "READY_FOR_DETECTION" && !arm_mission_completed_) {
                    home_ready_for_detection_ = true;
                    arm_ready_for_detection_ = false;

                    box_detection_count_ = 0;
                    box_confirmed_ = false;
                    have_recent_supply_box_class_ = false;
                    have_last_valid_box_time_ = false;
                    initial_search_active_ = false;
                    initial_search_timed_out_ = false;

                    RCLCPP_WARN(
                        this->get_logger(),
                        "[MISSION] ARM START HOME READY -> now starting arm D405 + supply YOLO");

                    if (local_perception_enabled_) {
                        if (!local_perception_started_) {
                            start_local_perception_stage();
                        }
                    }
                    else {
                        local_perception_ready_ = true;
                        start_detection_window_if_ready();
                    }
                }
            });

        // Real bridge readiness: AutoGrasp is not even launched until actual
        // /joint_states exist.
        arm_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                if (msg->name.empty() || msg->position.empty()) {
                    return;
                }

                // DROK_FRESH_JOINTSTATE_SOURCE_GUARD_V4_3_1
                if (arm_bridge_pid_ <= 0 || arm_startup_blocked_) {
                    return;
                }

                bool all_raw_seen = true;
                for (const bool seen : raw_feedback_seen_) {
                    if (!seen) {
                        all_raw_seen = false;
                        break;
                    }
                }
                if (!all_raw_seen) {
                    return;
                }

                const auto publisher_count =
                    this->count_publishers("/joint_states");
                if (publisher_count != 1) {
                    arm_startup_blocked_ = true;
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[ARM STARTUP BLOCKED] /joint_states publisher count=%zu; expected exactly 1.",
                        publisher_count);
                    return;
                }

                arm_joint_state_seen_ = true;
                arm_last_joint_state_time_ = this->now();

                if (!arm_bridge_ready_) {
                    arm_bridge_ready_ = true;
                    arm_bridge_ready_time_ = this->now();
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[ARM BRIDGE] FRESH Summer-owned /joint_states verified -> READY; AutoGrasp starts after %.2f sec",
                        arm_bridge_stable_sec_);
                }
});

        const std::array<std::string, 6> logical_joint_names = {
            "JOINT1", "JOINT2", "JOINT3", "JOINT4", "JOINT5", "JOINT6"};

        for (std::size_t i = 0;
             i < arm_feedback_topics_.size() && i < raw_feedback_seen_.size();
             ++i)
        {
            raw_feedback_subs_.push_back(
                this->create_subscription<std_msgs::msg::Float64>(
                    arm_feedback_topics_[i], 10,
                    [this, i](const std_msgs::msg::Float64::SharedPtr) {
                        raw_feedback_seen_[i] = true;
                    }));

            RCLCPP_INFO(
                this->get_logger(),
                "[ARM FEEDBACK WATCH] %s <- %s",
                logical_joint_names[i].c_str(),
                arm_feedback_topics_[i].c_str());
        }

        if (enable_camera_view_) {
            img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                camera_image_topic_, 10,
                std::bind(
                    &SummerNode::process_front,
                    this,
                    std::placeholders::_1));

            RCLCPP_WARN(
                this->get_logger(),
                "[CAMERA VIEW] ENABLED: %s",
                camera_image_topic_.c_str());
        }
        else {
            RCLCPP_INFO(
                this->get_logger(),
                "[CAMERA VIEW] DISABLED. YOLO topics are still used for grasping.");
        }

        // =====================================================================
        // PUBLISHERS
        // =====================================================================
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/ui_combined_vision", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/light_status", 10);
        arm_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/drok_arm_auto/enable", 10);

        // Existing chassis output remains 20 Hz. Summer only gates it while
        // ARM startup/search/grasp requires the chassis to be stationary.
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SummerNode::control_loop, this));

        // v5.0 ONE-COMMAND order:
        //   ARM bridge/TF -> /joint_states -> AutoGrasp -> START HOME
        //   -> READY_FOR_DETECTION -> LOCAL D405 + supply YOLO
        //   -> perception stable -> one 5-second box-search window.
        //
        // Camera/YOLO are intentionally NOT started before HOME anymore.
        // v5.1: stale bridge cleanup is internal to Summer, so the operator
        // only needs `ros2 run drokck summer_node`.
        cleanup_stale_arm_runtime_once();
        start_arm_runtime_stage1();
        arm_stage1_started_ = true;

        RCLCPP_WARN(
            this->get_logger(),
            "[MISSION] ONE NODE: chassis STOP -> ARM HOME -> arm D405+YOLO 5s -> optional GRASP -> DRIVE");

        RCLCPP_INFO(
            this->get_logger(),
            "[SUMMER] ONE NODE ONLY: ARM HOME -> D405+YOLO_supply 5s -> GRASP/HOLD -> RETURN HOME -> CAMERA OFF -> DRIVE");

        RCLCPP_INFO(
            this->get_logger(),
            "[NUC LOCAL] workspace=%s | arm D405+YOLO=%s",
            test_root_.c_str(),
            local_perception_enabled_ ? "ON" : "OFF");

        RCLCPP_INFO(
            this->get_logger(),
            "[ARM PATH] workspace=%s | run_real=%s | URDF=%s",
            arm_ws_root_.c_str(),
            (arm_ws_root_ + "/tools/run_real.sh").c_str(),
            arm_urdf_path_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "[D405 TF] %s -> %s | xyz=(%.5f %.5f %.5f) rpy=(%.3f %.3f %.3f)",
            arm_tf_parent_frame_.c_str(),
            arm_tf_child_frame_.c_str(),
            arm_tf_x_, arm_tf_y_, arm_tf_z_,
            arm_tf_roll_, arm_tf_pitch_, arm_tf_yaw_);
        RCLCPP_INFO(
            this->get_logger(),
            "[ARM TF] robot_state_publisher=%s | URDF=%s",
            arm_start_robot_state_publisher_ ? "Summer-managed" : "external",
            arm_urdf_path_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "[BOX] timeout=%.2fs required=%d min=%.3fm max=%.3fm association=%.3fs max_gap=%.3fs",
            box_search_timeout_sec_,
            box_detection_required_count_,
            box_min_camera_distance_m_,
            box_max_camera_distance_m_,
            class_xyz_association_max_sec_,
            box_detection_max_gap_sec_);
    }

    ~SummerNode() override {
        // Summer termination stops only the child processes Summer started.
        stop_child_process_group("AUTO_GRASP", arm_auto_pid_);
        stop_child_process_group("REAL_ARM_BRIDGE", arm_bridge_pid_);
        stop_child_process_group("ARM_ROBOT_STATE_PUBLISHER", arm_rsp_pid_);
        stop_child_process_group("D405_GRIPPER_STATIC_TF", tf_pid_);
        stop_child_process_group("LOCAL_YOLO", local_yolo_pid_);
        stop_child_process_group("LOCAL_D405", local_realsense_pid_);

        if (enable_camera_view_) {
            try {
                cv::destroyWindow("Summer_Vision");
            }
            catch (const cv::Exception &) {
            }
        }
    }

private:
    // =====================================================================
    // One-command startup cleanup
    // =====================================================================
    void cleanup_stale_arm_runtime_once() {
        // This runs BEFORE Summer spawns its own bridge/AutoGrasp children.
        // [x] patterns prevent pkill from matching the cleanup shell itself.
        const char *cmd =
            "pkill -INT -f '[m]otor_angle_publisher' 2>/dev/null || true; "
            "pkill -INT -f '[r]eal_joint_state_mapper.py' 2>/dev/null || true; "
            "pkill -INT -f '[m]oveit_to_rmd_bridge.py' 2>/dev/null || true; "
            "pkill -INT -f '[d]rok_auto_grasp_prototype1.py' 2>/dev/null || true; "
            "pkill -INT -f '[r]un_real.sh' 2>/dev/null || true; "
            "sleep 0.8; "
            "pkill -TERM -f '[m]otor_angle_publisher' 2>/dev/null || true; "
            "pkill -TERM -f '[r]eal_joint_state_mapper.py' 2>/dev/null || true; "
            "pkill -TERM -f '[m]oveit_to_rmd_bridge.py' 2>/dev/null || true; "
            "pkill -TERM -f '[d]rok_auto_grasp_prototype1.py' 2>/dev/null || true; "
            "pkill -TERM -f '[r]un_real.sh' 2>/dev/null || true; "
            "sleep 0.5";

        RCLCPP_INFO(
            this->get_logger(),
            "[STARTUP CLEANUP] clearing only stale DROK ARM bridge/AutoGrasp processes");

        const int rc = std::system(cmd);
        (void)rc;
    }

    // =====================================================================
    // Child process management
    // =====================================================================
    pid_t spawn_process(const std::string &name, const std::string &command) {
        const pid_t pid = fork();
        if (pid < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[ARM PROCESS] fork failed: %s",
                name.c_str());
            return -1;
        }

        if (pid == 0) {
            setpgid(0, 0);
            execl(
                "/bin/bash",
                "bash",
                "-lc",
                command.c_str(),
                static_cast<char *>(nullptr));
            _exit(127);
        }

        setpgid(pid, pid);
        RCLCPP_INFO(
            this->get_logger(),
            "[ARM PROCESS] started %-22s PID=%d",
            name.c_str(),
            static_cast<int>(pid));
        return pid;
    }

    void stop_child_process_group(const std::string &name, pid_t &pid) {
        if (pid <= 0) {
            return;
        }

        int status = 0;
        const pid_t result = waitpid(pid, &status, WNOHANG);
        if (result == pid) {
            pid = -1;
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[ARM PROCESS] stopping %s PID=%d",
            name.c_str(),
            static_cast<int>(pid));

        kill(-pid, SIGINT);
        usleep(300000);

        if (waitpid(pid, &status, WNOHANG) == 0) {
            kill(-pid, SIGTERM);
            usleep(300000);
        }

        if (waitpid(pid, &status, WNOHANG) == 0) {
            kill(-pid, SIGKILL);
        }

        waitpid(pid, &status, WNOHANG);
        pid = -1;
    }

    bool child_process_alive(const std::string &name, pid_t &pid) {
        if (pid <= 0) {
            return false;
        }

        int status = 0;
        const pid_t result = waitpid(pid, &status, WNOHANG);
        if (result == 0) {
            return true;
        }

        if (result == pid) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[ARM PROCESS] %s exited unexpectedly (PID=%d, status=%d)",
                name.c_str(),
                static_cast<int>(pid),
                status);
            pid = -1;
            if (!arm_mission_completed_) {
                arm_process_fault_ = true;
            }
            return false;
        }

        return true;
    }

    // =====================================================================
    // Local NUC arm-D405 + supply-YOLO startup
    // =====================================================================
    void start_local_perception_stage() {
        if (local_perception_started_ || local_perception_intentionally_stopped_) {
            return;
        }

        local_camera_color_seen_ = false;
        local_camera_depth_seen_ = false;
        local_camera_info_seen_ = false;
        local_perception_all_seen_started_ = false;
        local_perception_ready_ = false;
        local_perception_timeout_reported_ = false;

        const std::string camera_command =
            "source /opt/ros/humble/setup.bash && "
            "exec ros2 launch realsense2_camera rs_launch.py "
            "serial_no:=_" + d405_serial_ + " "
            "camera_namespace:=arm_d405 "
            "camera_name:=camera "
            "align_depth.enable:=true "
            "enable_color:=true "
            "enable_depth:=true "
            "pointcloud.enable:=false";

        const std::string yolo_command =
            "source /opt/ros/humble/setup.bash && "
            "source \"" + local_perception_ws_root_ + "/install/setup.bash\" && "
            "export DROK_YOLO_ENABLE_RTSP=0 && "
            "exec ros2 run yolo_xyz_publisher q --ros-args "
            "-r __node:=yolo_xyz_node_supply "
            "-r /camera/camera/color/image_raw:=/arm_d405/camera/color/image_raw "
            "-r /camera/camera/aligned_depth_to_color/image_raw:=/arm_d405/camera/aligned_depth_to_color/image_raw "
            "-r /camera/camera/color/camera_info:=/arm_d405/camera/color/camera_info "
            "-r /yolo_detected_object:=/yolo_detected_object_supply "
            "-r /yolo_object_xyz:=/yolo_object_xyz_supply";

        local_realsense_pid_ = spawn_process("LOCAL_D405", camera_command);
        local_yolo_pid_ = spawn_process("LOCAL_YOLO", yolo_command);
        local_perception_start_time_ = this->now();

        if (local_realsense_pid_ <= 0 || local_yolo_pid_ <= 0) {
            local_perception_started_ = false;
            RCLCPP_ERROR(
                this->get_logger(),
                "[LOCAL PERCEPTION] Failed to start D405 or supply YOLO -> skip ARM grasp phase and release chassis");
            finish_search_without_box("PERCEPTION_START_FAILED");
            return;
        }

        local_perception_started_ = true;

        RCLCPP_WARN(
            this->get_logger(),
            "[LOCAL PERCEPTION] D405 + supply YOLO started AFTER HOME. Waiting for color/depth/CameraInfo...");
    }

    void stop_local_perception_async(const std::string &reason) {
        if (!local_perception_enabled_ || local_perception_intentionally_stopped_) {
            return;
        }

        local_perception_intentionally_stopped_ = true;
        local_perception_ready_ = false;
        local_camera_color_seen_ = false;
        local_camera_depth_seen_ = false;
        local_camera_info_seen_ = false;

        RCLCPP_WARN(
            this->get_logger(),
            "[LOCAL PERCEPTION STOP] %s -> stopping ONLY LOCAL_YOLO + LOCAL_D405",
            reason.c_str());

        std::thread([this]() {
            stop_child_process_group("LOCAL_YOLO", local_yolo_pid_);
            stop_child_process_group("LOCAL_D405", local_realsense_pid_);
        }).detach();
    }

    void finish_search_without_box(const std::string &reason) {
        if (arm_mission_completed_ || box_confirmed_) {
            return;
        }

        initial_search_active_ = false;
        initial_search_timed_out_ = true;
        arm_ready_for_detection_ = false;
        arm_mission_completed_ = true;  // ARM phase is complete: no late trigger allowed.

        box_detection_count_ = 0;
        have_recent_supply_box_class_ = false;
        have_last_valid_box_time_ = false;

        stop_local_perception_async(reason);

        RCLCPP_WARN(
            this->get_logger(),
            "[MISSION] NO SUPPLY BOX (%s) -> D405/YOLO OFF -> ARM stays HOME -> chassis autonomous drive RESUME",
            reason.c_str());
        RCLCPP_WARN(
            this->get_logger(),
            "[MISSION] Late supply-box detections are disabled until Summer restarts.");
    }

    void start_detection_window_if_ready() {
        if (arm_mission_completed_ ||
            box_confirmed_ ||
            initial_search_active_ ||
            initial_search_timed_out_ ||
            !home_ready_for_detection_ ||
            !local_perception_ready_)
        {
            return;
        }

        box_detection_count_ = 0;
        have_recent_supply_box_class_ = false;
        have_last_valid_box_time_ = false;

        arm_ready_for_detection_ = true;
        initial_search_active_ = true;
        initial_search_start_time_ = this->now();

        RCLCPP_WARN(
            this->get_logger(),
            "[MISSION] D405 + supply YOLO READY -> stationary detection window START: %.2f sec",
            box_search_timeout_sec_);
        RCLCPP_INFO(
            this->get_logger(),
            "[BOX] valid range: %.3f m < distance < %.3f m",
            box_min_camera_distance_m_,
            box_max_camera_distance_m_);
    }

    void supervise_local_perception() {
        if (!local_perception_enabled_ || !local_perception_started_) {
            return;
        }

        // DROK_D405_SHUTDOWN_AFTER_GRASP_V3_9
        if (local_perception_intentionally_stopped_) {
            return;
        }

        const bool camera_alive =
            child_process_alive("LOCAL_D405", local_realsense_pid_);
        const bool yolo_alive =
            child_process_alive("LOCAL_YOLO", local_yolo_pid_);

        if (!camera_alive || !yolo_alive) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[LOCAL PERCEPTION] D405 or supply YOLO exited -> no-box path; chassis will be released");
            finish_search_without_box("PERCEPTION_PROCESS_EXIT");
            return;
        }

        // After readiness we still monitor both child processes every control
        // cycle; camera/YOLO loss is therefore visible to the same safety gate.
        if (local_perception_ready_) {
            return;
        }

        const bool all_seen =
            local_camera_color_seen_ &&
            local_camera_depth_seen_ &&
            local_camera_info_seen_;

        if (all_seen) {
            if (!local_perception_all_seen_started_) {
                local_perception_all_seen_started_ = true;
                local_perception_all_seen_time_ = this->now();
                RCLCPP_INFO(
                    this->get_logger(),
                    "[LOCAL PERCEPTION] color + aligned depth + CameraInfo received. Stabilizing %.2f sec...",
                    local_perception_stable_sec_);
            }

            const double stable_elapsed =
                (this->now() - local_perception_all_seen_time_).seconds();
            if (stable_elapsed >= local_perception_stable_sec_) {
                local_perception_ready_ = true;
                RCLCPP_WARN(
                    this->get_logger(),
                    "[LOCAL PERCEPTION READY] D405 topics stable + supply YOLO alive");
                start_detection_window_if_ready();
                return;
            }
        }

        const double elapsed =
            (this->now() - local_perception_start_time_).seconds();

        if (!local_perception_timeout_reported_ &&
            elapsed >= local_perception_ready_timeout_sec_)
        {
            local_perception_timeout_reported_ = true;
            RCLCPP_ERROR(
                this->get_logger(),
                "[LOCAL PERCEPTION] D405 not ready within %.1f sec. color=%s depth=%s info=%s -> skip grasp and release chassis",
                elapsed,
                local_camera_color_seen_ ? "OK" : "MISS",
                local_camera_depth_seen_ ? "OK" : "MISS",
                local_camera_info_seen_ ? "OK" : "MISS");
            finish_search_without_box("PERCEPTION_READY_TIMEOUT");
        }
    }

    // =====================================================================
    // ARM runtime startup
    // =====================================================================
    void start_arm_runtime_stage1() {
        // Static only between rigidly attached GRIPPER_BASE and D405 camera.
        // ARM_BASE_LINK -> GRIPPER_BASE remains dynamic through robot TF.
        std::ostringstream tf_command;
        tf_command << std::fixed << std::setprecision(9)
            << "source /opt/ros/humble/setup.bash && "
            << "exec ros2 run tf2_ros static_transform_publisher "
            << "--x " << arm_tf_x_
            << " --y " << arm_tf_y_
            << " --z " << arm_tf_z_
            << " --roll " << arm_tf_roll_
            << " --pitch " << arm_tf_pitch_
            << " --yaw " << arm_tf_yaw_
            << " --frame-id " << arm_tf_parent_frame_
            << " --child-frame-id " << arm_tf_child_frame_;

        tf_pid_ = spawn_process("D405_GRIPPER_STATIC_TF", tf_command.str());

        if (arm_start_robot_state_publisher_) {
            if (::access(arm_urdf_path_.c_str(), R_OK) != 0) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[ARM RSP BLOCK] URDF not readable: %s",
                    arm_urdf_path_.c_str());
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[ARM RSP BLOCK] Dynamic ARM_BASE_LINK -> GRIPPER_BASE TF cannot be generated. Chassis stays STOPPED.");
            }
            else {
                const std::string rsp_command =
                    "source /opt/ros/humble/setup.bash && "
                    "source \"" + arm_ws_root_ + "/install/setup.bash\" && "
                    "exec ros2 run robot_state_publisher robot_state_publisher "
                    "--ros-args -r __node:=drok_arm_real_state_publisher "
                    "-p robot_description:=\"$(cat \"" + arm_urdf_path_ + "\")\"";

                arm_rsp_pid_ = spawn_process(
                    "ARM_ROBOT_STATE_PUBLISHER",
                    rsp_command);
            }
        }

        // DROK_PRESPAWN_JOINTSTATE_GUARD_V4_3_1
        const auto preexisting_joint_publishers =
            this->count_publishers("/joint_states");

        if (preexisting_joint_publishers != 0) {
            arm_startup_blocked_ = true;
            RCLCPP_ERROR(
                this->get_logger(),
                "[ARM STARTUP BLOCKED] Found %zu pre-existing /joint_states publisher(s) BEFORE REAL_ARM_BRIDGE.",
                preexisting_joint_publishers);
            return;
        }

        arm_bridge_ready_ = false;
        arm_joint_state_seen_ = false;
        raw_feedback_seen_.fill(false);

        arm_bridge_pid_ = spawn_process(
            "REAL_ARM_BRIDGE",
            "exec bash \"" + arm_ws_root_ + "/tools/run_real.sh\"");

        arm_bridge_start_time_ = this->now();
        arm_last_diagnostic_time_ = this->now();

        RCLCPP_WARN(
            this->get_logger(),
            "[ARM STARTUP] Stage 1/2: arm TF + real bridge started. Waiting for /joint_states.");
    }

    void start_auto_grasp_if_ready() {
        if (arm_auto_started_ || !arm_bridge_ready_) {
            return;
        }

        const double stable_elapsed =
            (this->now() - arm_bridge_ready_time_).seconds();
        if (stable_elapsed < arm_bridge_stable_sec_) {
            return;
        }

        if (!child_process_alive("REAL_ARM_BRIDGE", arm_bridge_pid_)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[ARM STARTUP] Bridge died before AutoGrasp start. Chassis remains STOP.");
            return;
        }

        if (arm_start_robot_state_publisher_ &&
            !child_process_alive("ARM_ROBOT_STATE_PUBLISHER", arm_rsp_pid_))
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "[ARM STARTUP] robot_state_publisher is not alive. Dynamic camera TF unavailable; AutoGrasp will NOT start.");
            return;
        }

        const std::string auto_grasp_script =
            arm_ws_root_ + "/tools/drok_auto_grasp_prototype1.py";

        const std::string auto_grasp_command =
            "source /opt/ros/humble/setup.bash && "
            "source \"" + arm_ws_root_ + "/install/setup.bash\" && "
            "export DROK_ARM_WS=\"" + arm_ws_root_ + "\" && "
            "export DROK_GRASP_WS=\"" + arm_ws_root_ + "\" && "
            "export DROK_V11_CORE_PATH=\"" +
                arm_ws_root_ + "/tools/interactive_box_ik_grasp_v11.py\" && "
            "export DROK_HOME_Q_FILE=\"" +
                arm_ws_root_ + "/config/home_q.json\" && "
            "export DROK_RETURN_HOME_Q_FILE=\"" +
                arm_ws_root_ + "/config/return_home_q.json\" && "
            "unset DROK_V11_PATH || true; "
            "exec python3 \"" + auto_grasp_script + "\" --ros-args "
            "-p require_manual_j1_j6_preposition:=false "
            "-r /yolo_detected_object:=/yolo_detected_object_supply "
            "-r /yolo_object_xyz:=/yolo_object_xyz_supply";

        arm_auto_pid_ = spawn_process(
            "AUTO_GRASP",
            auto_grasp_command);

        if (arm_auto_pid_ > 0) {
            arm_auto_started_ = true;
            RCLCPP_WARN(
                this->get_logger(),
                "[ARM STARTUP] Stage 2/2: AutoGrasp started AFTER verified /joint_states.");
            RCLCPP_WARN(
                this->get_logger(),
                "[ARM HOME POLICY] legacy manual J1 preposition gate DISABLED -> current q will move to captured START HOME by the existing Poly5 HOME motion");
        }
    }

    void log_bridge_feedback_diagnostic() {
        const std::array<std::string, 6> names = {
            "JOINT1", "JOINT2", "JOINT3", "JOINT4", "JOINT5", "JOINT6"};

        std::ostringstream missing;
        bool first = true;
        for (std::size_t i = 0; i < raw_feedback_seen_.size(); ++i) {
            if (!raw_feedback_seen_[i]) {
                if (!first) {
                    missing << ", ";
                }
                first = false;
                missing << names[i] << "=" << arm_feedback_topics_[i];
            }
        }

        if (first) {
            RCLCPP_WARN(
                this->get_logger(),
                "[ARM BRIDGE WAIT] All 6 raw motor topics were seen, but /joint_states is still missing. Check real_joint_state_mapper.");
        }
        else {
            RCLCPP_WARN(
                this->get_logger(),
                "[ARM BRIDGE WAIT] /joint_states missing. Raw feedback NOT seen: %s",
                missing.str().c_str());
        }
    }

    void supervise_arm_runtime() {
        // ARM startup is independent from D405/YOLO. HOME must happen first.
        if (!arm_stage1_started_ && !arm_startup_blocked_) {
            start_arm_runtime_stage1();
            arm_stage1_started_ = true;
        }

        if (!arm_stage1_started_) {
            return;
        }

        // D405/YOLO supervision becomes active only after AutoGrasp HOME READY
        // has started the local perception stage.
        supervise_local_perception();
        start_detection_window_if_ready();

        child_process_alive("D405_GRIPPER_STATIC_TF", tf_pid_);
        if (arm_start_robot_state_publisher_) {
            child_process_alive("ARM_ROBOT_STATE_PUBLISHER", arm_rsp_pid_);
        }
        child_process_alive("REAL_ARM_BRIDGE", arm_bridge_pid_);
        if (arm_auto_started_) {
            child_process_alive("AUTO_GRASP", arm_auto_pid_);
        }

        if (!arm_auto_started_) {
            start_auto_grasp_if_ready();
        }

        if (!arm_bridge_ready_) {
            const double elapsed =
                (this->now() - arm_bridge_start_time_).seconds();
            const double since_diag =
                (this->now() - arm_last_diagnostic_time_).seconds();

            if (since_diag >= 3.0) {
                log_bridge_feedback_diagnostic();
                arm_last_diagnostic_time_ = this->now();
            }

            if (!arm_bridge_timeout_reported_ &&
                elapsed >= arm_bridge_ready_timeout_sec_)
            {
                arm_bridge_timeout_reported_ = true;
                arm_startup_blocked_ = true;
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[ARM STARTUP BLOCKED] No /joint_states for %.1f sec. AutoGrasp was NOT started; no HOME/gripper/motion command was sent.",
                    elapsed);
            }
        }

        // Once real feedback has been established, do not allow autonomous
        // driving or a new grasp trigger if /joint_states goes stale.
        arm_feedback_stale_ = false;
        if (arm_bridge_ready_ && !arm_mission_completed_) {
            const double feedback_age =
                (this->now() - arm_last_joint_state_time_).seconds();
            if (feedback_age > arm_joint_state_stale_timeout_sec_) {
                arm_feedback_stale_ = true;
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "[ARM FEEDBACK LOST] /joint_states stale for %.2f sec (limit %.2f sec). Chassis STOP + ARM trigger blocked.",
                    feedback_age,
                    arm_joint_state_stale_timeout_sec_);
            }
        }
    }

    // =====================================================================
    // Initial 5-second stationary search
    // =====================================================================
    void update_initial_search_window() {
        if (!initial_search_active_ ||
            !arm_ready_for_detection_ ||
            box_confirmed_ ||
            arm_mission_completed_)
        {
            return;
        }

        const double elapsed =
            (this->now() - initial_search_start_time_).seconds();

        if (elapsed < box_search_timeout_sec_) {
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "[BOX] %.2f sec detection window expired with no confirmed supply_box",
            box_search_timeout_sec_);

        finish_search_without_box("5SEC_NO_BOX");
    }

    // =====================================================================
    // supply_box validation
    // =====================================================================
    void process_supply_box_xyz(
        const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        if (!arm_ready_for_detection_ ||
            !initial_search_active_ ||
            arm_mission_completed_)
        {
            return;
        }

        if (is_waiting_ || arm_started_ || box_confirmed_) {
            return;
        }

        if (msg->header.frame_id != arm_tf_child_frame_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "[BOX] Ignoring /yolo_object_xyz_supply frame_id='%s'; expected '%s'",
                msg->header.frame_id.c_str(),
                arm_tf_child_frame_.c_str());
            box_detection_count_ = 0;
            return;
        }

        if (!have_recent_supply_box_class_) {
            box_detection_count_ = 0;
            return;
        }

        const double class_age =
            (this->now() - last_supply_box_class_time_).seconds();

        if (class_age < 0.0 || class_age > class_xyz_association_max_sec_) {
            box_detection_count_ = 0;
            return;
        }

        const double x = msg->vector.x;
        const double y = msg->vector.y;
        const double z = msg->vector.z;
        const double camera_distance = std::sqrt(x * x + y * y + z * z);

        if (!std::isfinite(camera_distance)) {
            box_detection_count_ = 0;
            return;
        }

        if (camera_distance <= box_min_camera_distance_m_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[BOX] REJECT too close: %.3f m <= min %.3f m",
                camera_distance,
                box_min_camera_distance_m_);
            box_detection_count_ = 0;
            return;
        }

        if (camera_distance >= box_max_camera_distance_m_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[BOX] REJECT too far: %.3f m >= max %.3f m",
                camera_distance,
                box_max_camera_distance_m_);
            box_detection_count_ = 0;
            return;
        }

        const auto now = this->now();
        if (have_last_valid_box_time_) {
            const double gap = (now - last_valid_box_time_).seconds();
            if (gap < 0.0 || gap > box_detection_max_gap_sec_) {
                box_detection_count_ = 0;
            }
        }
        last_valid_box_time_ = now;
        have_last_valid_box_time_ = true;

        ++box_detection_count_;

        RCLCPP_INFO(
            this->get_logger(),
            "[BOX] valid %d/%d | distance=%.3f m | xyz=(%+.3f, %+.3f, %+.3f)",
            box_detection_count_,
            box_detection_required_count_,
            camera_distance,
            x, y, z);

        if (box_detection_count_ >= box_detection_required_count_) {
            box_confirmed_ = true;
            arm_ready_for_detection_ = false;
            initial_search_active_ = false;

            RCLCPP_WARN(
                this->get_logger(),
                "[MISSION] supply_box VALID inside 5-second window -> LATCH GRASP; D405/YOLO remain ON through RETURN HOME");
            start_arm_sequence_if_needed();
        }
    }

    // =====================================================================
    // ARM mission state
    // =====================================================================
    void start_arm_sequence_if_needed() {
        if (is_waiting_ || arm_started_ || arm_mission_completed_) {
            return;
        }

        is_waiting_ = true;
        wait_start_time_ = this->now();

        RCLCPP_WARN(
            this->get_logger(),
            "[ARM] supply_box CONFIRMED -> chassis STOP -> prepare one grasp mission");
    }

    void maybe_start_arm_after_chassis_settle() {
        if (!is_waiting_ || arm_started_ || arm_mission_completed_) {
            return;
        }

        const double elapsed =
            (this->now() - wait_start_time_).seconds();

        if (elapsed < arm_chassis_settle_sec_) {
            return;
        }

        if (arm_startup_blocked_ || arm_process_fault_ || arm_feedback_stale_) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[ARM BLOCK] ARM runtime is not healthy. Grasp enable will not be published; chassis remains STOPPED.");
            return;
        }

        if (!arm_auto_started_ ||
            !child_process_alive("AUTO_GRASP", arm_auto_pid_))
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[ARM BLOCK] AutoGrasp is not alive. Chassis remains STOPPED.");
            return;
        }

        auto enable_msg = std_msgs::msg::Bool();
        enable_msg.data = true;
        arm_enable_pub_->publish(enable_msg);
        arm_started_ = true;

        RCLCPP_WARN(
            this->get_logger(),
            "[ARM] chassis settled %.2f sec -> /drok_arm_auto/enable=True ONCE",
            arm_chassis_settle_sec_);
    }

    // =====================================================================
    // Chassis control - existing navigation output is only gated, never altered
    // =====================================================================
    void control_loop() {
        supervise_arm_runtime();
        update_initial_search_window();

        auto out_msg = geometry_msgs::msg::Twist();

        // STOP conditions only. Outside these conditions, the exact existing
        // /cmd_vel_nav message is passed through untouched.
        const bool startup_not_ready =
            (!arm_ready_for_detection_ &&
             !arm_mission_completed_ &&
             !initial_search_timed_out_ &&
             !is_waiting_ &&
             !box_confirmed_);

        const bool arm_safety_block =
            (!arm_mission_completed_) &&
            (arm_startup_blocked_ || arm_process_fault_ || arm_feedback_stale_);

        if (is_red_light_ ||
            is_waiting_ ||
            startup_not_ready ||
            initial_search_active_ ||
            arm_safety_block)
        {
            out_msg.linear.x = 0.0;
            out_msg.linear.y = 0.0;
            out_msg.linear.z = 0.0;
            out_msg.angular.x = 0.0;
            out_msg.angular.y = 0.0;
            out_msg.angular.z = 0.0;
        }
        else {
            out_msg = current_nav_;
        }

        maybe_start_arm_after_chassis_settle();
        cmd_pub_->publish(out_msg);
    }

    // =====================================================================
    // Optional original Summer camera visualization
    // =====================================================================
    void process_front(const sensor_msgs::msg::Image::SharedPtr mf) {
        if (!enable_camera_view_) {
            return;
        }

        try {
            cv::Mat img_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
            cv::resize(img_f, img_f, cv::Size(640, 480));

            cv::Mat ui_view;
            cv::resize(img_f, ui_view, cv::Size(), 0.5, 0.5);

            auto ui_msg = cv_bridge::CvImage(
                mf->header,
                "bgr8",
                ui_view).toImageMsg();
            ui_image_pub_->publish(*ui_msg);

            cv::imshow("Summer_Vision", ui_view);
            cv::waitKey(1);
        }
        catch (const cv::Exception &e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "OpenCV Error: %s",
                e.what());
        }
    }

    // =====================================================================
    // Members
    // =====================================================================
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr local_color_ready_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr local_depth_ready_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr local_info_ready_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_bbox_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_object_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yolo_xyz_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_done_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> raw_feedback_subs_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ui_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_enable_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::Twist current_nav_;

    // Existing Summer state.
    bool is_red_light_ = false;
    bool is_waiting_ = false;
    rclcpp::Time wait_start_time_;

    // ARM mission one-shot state for this Summer process.
    bool arm_started_ = false;
    bool arm_mission_completed_ = false;
    bool arm_ready_for_detection_ = false;
    bool home_ready_for_detection_ = false;

    // Initial stationary search state.
    bool initial_search_active_ = false;
    bool initial_search_timed_out_ = false;
    rclcpp::Time initial_search_start_time_;
    double box_search_timeout_sec_ = 5.0;

    // supply_box validation.
    int box_detection_required_count_ = 5;
    int box_detection_count_ = 0;
    double box_min_camera_distance_m_ = 0.20;
    double box_max_camera_distance_m_ = 1.00;
    double class_xyz_association_max_sec_ = 0.25;
    double box_detection_max_gap_sec_ = 0.40;
    bool box_confirmed_ = false;
    bool have_recent_supply_box_class_ = false;
    rclcpp::Time last_supply_box_class_time_;
    bool have_last_valid_box_time_ = false;
    rclcpp::Time last_valid_box_time_;

    // Optional original camera view.
    bool enable_camera_view_ = false;
    std::string camera_image_topic_ = "/camera/camera/color/image_raw";

    // Laptop-local perception test runtime.
    std::string test_root_;
    bool local_perception_enabled_ = true;
    std::string local_perception_ws_root_;
    std::string local_camera_color_topic_;
    std::string local_camera_depth_topic_;
    std::string local_camera_info_topic_;
    double local_perception_ready_timeout_sec_ = 20.0;
    double local_perception_stable_sec_ = 0.75;
    std::string d405_serial_ = "260322271364";
    pid_t local_realsense_pid_ = -1;
    pid_t local_yolo_pid_ = -1;
    bool local_camera_color_seen_ = false;
    bool local_camera_depth_seen_ = false;
    bool local_camera_info_seen_ = false;
    bool local_perception_all_seen_started_ = false;
    bool local_perception_started_ = false;
    bool local_perception_ready_ = false;
    bool local_perception_timeout_reported_ = false;
    bool local_perception_intentionally_stopped_ = false;
    rclcpp::Time local_perception_start_time_;
    rclcpp::Time local_perception_all_seen_time_;

    // ARM runtime.
    std::string arm_ws_root_;
    std::string arm_auto_runner_path_;
    bool arm_stage1_started_ = false;
    pid_t tf_pid_ = -1;
    pid_t arm_rsp_pid_ = -1;
    pid_t arm_bridge_pid_ = -1;
    pid_t arm_auto_pid_ = -1;
    bool arm_auto_started_ = false;
    bool arm_start_robot_state_publisher_ = true;
    std::string arm_urdf_path_;
    bool arm_bridge_ready_ = false;
    bool arm_joint_state_seen_ = false;
    bool arm_bridge_timeout_reported_ = false;
    bool arm_startup_blocked_ = false;
    bool arm_process_fault_ = false;
    bool arm_feedback_stale_ = false;
    rclcpp::Time arm_bridge_start_time_;
    rclcpp::Time arm_bridge_ready_time_;
    rclcpp::Time arm_last_joint_state_time_;
    rclcpp::Time arm_last_diagnostic_time_;
    double arm_bridge_ready_timeout_sec_ = 15.0;
    double arm_bridge_stable_sec_ = 0.5;
    double arm_joint_state_stale_timeout_sec_ = 1.0;
    std::vector<std::string> arm_feedback_topics_;
    std::array<bool, 6> raw_feedback_seen_ = {
        false, false, false, false, false, false};

    // D405 rigid transform / arm trigger timing.
    double arm_tf_x_ = 0.0;
    double arm_tf_y_ = 0.0;
    double arm_tf_z_ = 0.0;
    double arm_tf_roll_ = 0.0;
    double arm_tf_pitch_ = 0.0;
    double arm_tf_yaw_ = 0.0;
    std::string arm_tf_parent_frame_;
    std::string arm_tf_child_frame_;
    double arm_chassis_settle_sec_ = 0.5;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SummerNode>());
    rclcpp::shutdown();
    return 0;
}
