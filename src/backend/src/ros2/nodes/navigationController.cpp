/**
 * @file navigationController.cpp
 * @version 1.0.0
 * @brief Enhanced ROS 2 node for autonomous navigation control of agricultural robots
 * @copyright Copyright (c) 2024
 */

#include <rclcpp/rclcpp.hpp>  // v2.4.0
#include <nav2_msgs/msg/path.hpp>  // v1.0.0
#include <geometry_msgs/msg/point.hpp>  // v4.2.3
#include <tf2_ros/transform_broadcaster.hpp>  // v0.25.1
#include <chrono>
#include <memory>
#include <map>
#include <vector>
#include <stdexcept>

using namespace std::chrono_literals;

class NavigationController : public rclcpp::Node {
public:
    explicit NavigationController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("navigation_controller", options) {
        initialize();
    }

private:
    // Subscriptions
    rclcpp::Subscription<MissionCommand>::SharedPtr mission_sub_;
    rclcpp::Subscription<DeviceStatus>::SharedPtr status_sub_;
    
    // Publishers
    rclcpp::Publisher<nav2_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<DeviceStatus>::SharedPtr status_pub_;
    
    // State management
    std::map<std::string, DeviceStatus> robot_states_;
    std::mutex state_mutex_;
    
    // Performance monitoring
    struct LatencyMetrics {
        std::chrono::high_resolution_clock::time_point start_time;
        double processing_time_ms;
    };
    std::map<std::string, LatencyMetrics> command_latencies_;

    void initialize() {
        // Configure QoS settings for reliability
        auto qos_reliable = rclcpp::QoS(10)
            .reliable()
            .transient_local()
            .keep_last(10);

        // Initialize subscriptions with QoS profiles
        mission_sub_ = create_subscription<MissionCommand>(
            "mission_commands",
            qos_reliable,
            std::bind(&NavigationController::missionCommandCallback, this, std::placeholders::_1)
        );

        status_sub_ = create_subscription<DeviceStatus>(
            "device_status",
            qos_reliable,
            std::bind(&NavigationController::deviceStatusCallback, this, std::placeholders::_1)
        );

        // Initialize publishers with QoS profiles
        path_pub_ = create_publisher<nav2_msgs::msg::Path>(
            "navigation_path",
            qos_reliable
        );

        status_pub_ = create_publisher<DeviceStatus>(
            "controller_status",
            qos_reliable
        );

        // Initialize parameters
        declare_parameter("max_fleet_size", 24);
        declare_parameter("command_timeout_ms", 100);
        declare_parameter("path_planning_resolution", 0.5);
        
        // Initialize health monitoring timer
        health_check_timer_ = create_wall_timer(
            1s,
            std::bind(&NavigationController::performHealthCheck, this)
        );

        RCLCPP_INFO(get_logger(), "Navigation Controller initialized successfully");
    }

    void missionCommandCallback(const MissionCommand::SharedPtr msg) {
        try {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            // Validate command integrity
            if (!validateMissionCommand(msg)) {
                RCLCPP_ERROR(get_logger(), "Invalid mission command received");
                return;
            }

            // Process command based on type
            switch (msg->command_type) {
                case MissionCommand::COMMAND_TYPE_START:
                    processStartCommand(msg);
                    break;
                case MissionCommand::COMMAND_TYPE_PAUSE:
                    processPauseCommand(msg);
                    break;
                case MissionCommand::COMMAND_TYPE_RESUME:
                    processResumeCommand(msg);
                    break;
                case MissionCommand::COMMAND_TYPE_ABORT:
                    processAbortCommand(msg);
                    break;
                default:
                    RCLCPP_WARN(get_logger(), "Unsupported command type: %d", msg->command_type);
                    return;
            }

            // Monitor processing latency
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            if (duration.count() > get_parameter("command_timeout_ms").as_int()) {
                RCLCPP_WARN(get_logger(), "Command processing exceeded timeout: %ld ms", duration.count());
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error processing mission command: %s", e.what());
            publishErrorStatus(msg->mission_id, "Command processing failed");
        }
    }

    void deviceStatusCallback(const DeviceStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        try {
            // Update robot state
            robot_states_[msg->device_id] = *msg;

            // Check for critical status changes
            if (msg->status == DeviceStatus::ROBOT_STATUS_ERROR) {
                RCLCPP_ERROR(get_logger(), "Device %s reported error: %s", 
                    msg->device_id.c_str(), msg->diagnostic_info.c_str());
                handleDeviceError(msg);
            }

            // Monitor battery levels
            if (msg->battery_level < 20.0) {
                RCLCPP_WARN(get_logger(), "Low battery warning for device %s: %.1f%%",
                    msg->device_id.c_str(), msg->battery_level);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error processing device status: %s", e.what());
        }
    }

    bool validateMissionCommand(const MissionCommand::SharedPtr msg) {
        if (msg->device_ids.size() > get_parameter("max_fleet_size").as_int()) {
            RCLCPP_ERROR(get_logger(), "Fleet size exceeds maximum limit");
            return false;
        }

        if (msg->coverage_area.empty()) {
            RCLCPP_ERROR(get_logger(), "Empty coverage area in mission command");
            return false;
        }

        if (msg->version != MissionCommand::VERSION) {
            RCLCPP_ERROR(get_logger(), "Unsupported message version");
            return false;
        }

        return true;
    }

    void processStartCommand(const MissionCommand::SharedPtr msg) {
        // Generate optimized path plan
        auto path = generatePathPlan(msg->coverage_area);
        
        // Assign waypoints to robots
        auto assignments = assignWaypoints(path, msg->device_ids);
        
        // Publish navigation paths
        for (const auto& assignment : assignments) {
            publishNavigationPath(assignment);
        }

        RCLCPP_INFO(get_logger(), "Started mission %s with %zu devices",
            msg->mission_id.c_str(), msg->device_ids.size());
    }

    void performHealthCheck() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        for (const auto& [device_id, status] : robot_states_) {
            auto now = this->now();
            auto last_update = rclcpp::Time(status.last_active);
            
            if ((now - last_update).seconds() > 5.0) {
                RCLCPP_WARN(get_logger(), "Device %s hasn't reported status for >5s",
                    device_id.c_str());
            }
        }
    }

    void handleDeviceError(const DeviceStatus::SharedPtr msg) {
        // Implement error recovery logic
        auto recovery_status = attemptErrorRecovery(msg);
        
        if (!recovery_status) {
            RCLCPP_ERROR(get_logger(), "Error recovery failed for device %s",
                msg->device_id.c_str());
            // Trigger emergency protocols if needed
        }
    }

    rclcpp::TimerBase::SharedPtr health_check_timer_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavigationController)