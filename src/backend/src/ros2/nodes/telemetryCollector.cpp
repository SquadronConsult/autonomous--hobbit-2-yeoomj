// Copyright 2024 Agricultural Management System
// ROS 2 node for high-performance telemetry collection with health monitoring
// Version: 1.0.0
// Dependencies:
// - rclcpp 2.4.0
// - std_msgs 4.2.3
// - geometry_msgs 4.2.3
// - message_filters 4.2.3

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <chrono>
#include <memory>
#include <vector>
#include <unordered_map>

#include "../msgs/TelemetryData.hpp"
#include "../msgs/DeviceStatus.hpp"

using namespace std::chrono_literals;

// Global constants
constexpr const char* TELEMETRY_TOPIC = "/telemetry/data";
constexpr const char* DEVICE_STATUS_TOPIC = "/device/status";
constexpr int PUBLISH_RATE_HZ = 10;
constexpr int BATCH_SIZE = 100;
constexpr int MAX_LATENCY_MS = 100;
constexpr float CRITICAL_BATTERY_THRESHOLD = 0.15f;
constexpr int MAX_RETRY_ATTEMPTS = 3;
constexpr int HEALTH_CHECK_INTERVAL_MS = 1000;

class TelemetryCollector : public rclcpp::Node {
public:
    TelemetryCollector()
        : Node("telemetry_collector")
    {
        initializeParameters();
        initializeSubscriptions();
        initializePublishers();
        initializeHealthMonitoring();
    }

private:
    // Subscriptions
    rclcpp::Subscription<TelemetryData>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<DeviceStatus>::SharedPtr status_sub_;
    
    // Publishers
    rclcpp::Publisher<TelemetryData>::SharedPtr telemetry_pub_;
    
    // Data management
    std::vector<TelemetryData> batch_queue_;
    rclcpp::TimerBase::SharedPtr batch_timer_;
    std::unordered_map<std::string, DeviceStatus> device_cache_;
    
    // Performance tracking
    std::chrono::steady_clock::time_point last_process_time_;
    int processed_messages_ = 0;
    int dropped_messages_ = 0;

    void initializeParameters() {
        this->declare_parameter("batch_size", BATCH_SIZE);
        this->declare_parameter("publish_rate", PUBLISH_RATE_HZ);
        this->declare_parameter("max_latency", MAX_LATENCY_MS);
    }

    void initializeSubscriptions() {
        // Configure QoS for reliability and low latency
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        qos_profile.reliable().deadline(std::chrono::milliseconds(MAX_LATENCY_MS));

        // Telemetry subscription
        auto telemetry_callback = [this](const TelemetryData::SharedPtr msg) {
            this->telemetryCallback(msg);
        };
        telemetry_sub_ = this->create_subscription<TelemetryData>(
            TELEMETRY_TOPIC, qos_profile, telemetry_callback);

        // Device status subscription
        auto status_callback = [this](const DeviceStatus::SharedPtr msg) {
            this->deviceStatusCallback(msg);
        };
        status_sub_ = this->create_subscription<DeviceStatus>(
            DEVICE_STATUS_TOPIC, qos_profile, status_callback);
    }

    void initializePublishers() {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        qos_profile.reliable().deadline(std::chrono::milliseconds(MAX_LATENCY_MS));

        telemetry_pub_ = this->create_publisher<TelemetryData>(
            "/telemetry/processed", qos_profile);

        // Initialize batch processing timer
        batch_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / PUBLISH_RATE_HZ),
            std::bind(&TelemetryCollector::processBatch, this));
    }

    void initializeHealthMonitoring() {
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(HEALTH_CHECK_INTERVAL_MS),
            std::bind(&TelemetryCollector::checkSystemHealth, this));
    }

    void telemetryCallback(const TelemetryData::SharedPtr msg) {
        auto now = std::chrono::steady_clock::now();
        auto latency = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(msg->timestamp.nanosec))).count();

        // Check latency threshold
        if (latency > MAX_LATENCY_MS) {
            RCLCPP_WARN(this->get_logger(), 
                "High latency detected: %ld ms for device %s", 
                latency, msg->device_id.c_str());
        }

        // Process critical status immediately
        if (msg->status == TelemetryData::STATUS_CRITICAL) {
            processCriticalTelemetry(msg);
            return;
        }

        // Add to batch queue
        batch_queue_.push_back(*msg);
        if (batch_queue_.size() >= static_cast<size_t>(BATCH_SIZE)) {
            processBatch();
        }

        processed_messages_++;
    }

    void deviceStatusCallback(const DeviceStatus::SharedPtr msg) {
        device_cache_[msg->device_id] = *msg;

        // Check for critical battery level
        if (msg->battery_level < CRITICAL_BATTERY_THRESHOLD) {
            RCLCPP_WARN(this->get_logger(),
                "Critical battery level for device %s: %.2f%%",
                msg->device_id.c_str(), msg->battery_level * 100.0f);
            initiateRecoveryProcedure(msg->device_id);
        }

        // Monitor device health
        if (msg->status == DeviceStatus::ROBOT_STATUS_ERROR) {
            RCLCPP_ERROR(this->get_logger(),
                "Device %s reported error: %d",
                msg->device_id.c_str(), msg->error_code);
            handleDeviceError(msg);
        }
    }

    void processBatch() {
        if (batch_queue_.empty()) {
            return;
        }

        // Process and publish batch
        for (const auto& telemetry : batch_queue_) {
            telemetry_pub_->publish(telemetry);
        }

        // Clear batch queue
        batch_queue_.clear();

        // Update performance metrics
        updatePerformanceMetrics();
    }

    void processCriticalTelemetry(const TelemetryData::SharedPtr msg) {
        // Immediate processing for critical messages
        telemetry_pub_->publish(*msg);
        RCLCPP_INFO(this->get_logger(),
            "Critical telemetry processed for device %s",
            msg->device_id.c_str());
    }

    void handleDeviceError(const DeviceStatus::SharedPtr msg) {
        // Implement error handling logic
        RCLCPP_ERROR(this->get_logger(),
            "Device error detected: %s, Code: %d",
            msg->device_id.c_str(), msg->error_code);
    }

    void initiateRecoveryProcedure(const std::string& device_id) {
        // Implement recovery logic
        RCLCPP_INFO(this->get_logger(),
            "Initiating recovery procedure for device %s",
            device_id.c_str());
    }

    void checkSystemHealth() {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_process_time_).count();

        if (duration > 0) {
            float message_rate = static_cast<float>(processed_messages_) / duration;
            RCLCPP_INFO(this->get_logger(),
                "Processing rate: %.2f msg/s, Dropped: %d",
                message_rate, dropped_messages_);
        }
    }

    void updatePerformanceMetrics() {
        last_process_time_ = std::chrono::steady_clock::now();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryCollector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}