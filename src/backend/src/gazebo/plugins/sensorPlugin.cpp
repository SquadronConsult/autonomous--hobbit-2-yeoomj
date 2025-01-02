// NVIDIA DeepStream Agricultural Management System
// Gazebo Sensor Plugin Implementation
// Version: 1.0.0
// Dependencies:
// - gazebo-dev v11.0
// - ros2-iron
// - sensor_msgs (ROS 2 Iron)
// - geometry_msgs (ROS 2 Iron)
// - diagnostic_msgs (ROS 2 Iron)

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "TelemetryData.msg"

// Global constants for sensor configuration
constexpr double CAMERA_UPDATE_RATE = 30.0;
constexpr double IMU_UPDATE_RATE = 100.0;
constexpr double GPS_UPDATE_RATE = 10.0;
constexpr size_t IMAGE_POOL_SIZE = 10;
constexpr double DIAGNOSTIC_PERIOD = 1.0;
constexpr double MAX_SENSOR_TIMEOUT = 5.0;

namespace gazebo {

// Message pool for zero-copy publishing
template<typename T>
class MessagePool {
public:
    MessagePool(size_t size) : pool_size_(size) {
        for (size_t i = 0; i < size; ++i) {
            pool_.push_back(std::make_shared<T>());
        }
    }

    typename std::shared_ptr<T> acquire() {
        if (available_.empty()) {
            return std::make_shared<T>();
        }
        auto msg = available_.front();
        available_.pop_front();
        return msg;
    }

    void release(typename std::shared_ptr<T> msg) {
        if (available_.size() < pool_size_) {
            available_.push_back(msg);
        }
    }

private:
    size_t pool_size_;
    std::deque<typename std::shared_ptr<T>> pool_;
    std::deque<typename std::shared_ptr<T>> available_;
};

class SensorPlugin : public SensorPlugin {
public:
    SensorPlugin() : node_(std::make_shared<rclcpp::Node>("sensor_plugin")) {}

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {
        sensor_ = _sensor;
        
        // Initialize ROS 2 publishers with QoS settings
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliable()
            .durability_volatile();

        telemetry_pub_ = node_->create_publisher<TelemetryData>(
            "telemetry", qos);

        // Setup sensor-specific handlers
        if (auto camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor_)) {
            setupCamera(camera, _sdf);
        } else if (auto imu = std::dynamic_pointer_cast<sensors::ImuSensor>(sensor_)) {
            setupImu(imu, _sdf);
        } else if (auto gps = std::dynamic_pointer_cast<sensors::GpsSensor>(sensor_)) {
            setupGps(gps, _sdf);
        }

        // Initialize diagnostics
        diagnostic_timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(DIAGNOSTIC_PERIOD),
            std::bind(&SensorPlugin::publishDiagnostics, this));

        last_update_ = node_->now();
    }

private:
    void setupCamera(sensors::CameraSensorPtr camera, sdf::ElementPtr sdf) {
        camera_ = camera;
        
        // Configure camera-specific publishers
        image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            "camera/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        // Initialize message pool
        image_pool_ = std::make_unique<MessagePool<sensor_msgs::msg::Image>>(IMAGE_POOL_SIZE);

        // Setup camera update connection
        camera_->SetUpdateRate(CAMERA_UPDATE_RATE);
        update_conn_ = camera_->ConnectNewImageFrame(
            std::bind(&SensorPlugin::onNewFrame, this,
                     std::placeholders::_1, std::placeholders::_2,
                     std::placeholders::_3, std::placeholders::_4,
                     std::placeholders::_5));

        // Configure noise model if specified
        if (sdf->HasElement("noise")) {
            auto noise = sdf->GetElement("noise");
            camera_->SetNoise(noise->Get<double>("mean"),
                            noise->Get<double>("stddev"));
        }
    }

    void onNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format) {
        auto msg = image_pool_->acquire();
        
        // Fill image message
        msg->header.stamp = node_->now();
        msg->header.frame_id = sensor_->ParentName();
        msg->height = _height;
        msg->width = _width;
        msg->encoding = _format;
        msg->is_bigendian = false;
        msg->step = _width * _depth;
        
        // Zero-copy when possible
        msg->data.assign(_image, _image + _width * _height * _depth);

        // Publish image
        image_pub_->publish(std::move(*msg));
        image_pool_->release(msg);

        // Update telemetry
        publishTelemetry(TelemetryData::TELEMETRY_TYPE_SENSOR);
        last_update_ = node_->now();
    }

    void publishTelemetry(uint8_t type) {
        auto msg = std::make_unique<TelemetryData>();
        msg->header.stamp = node_->now();
        msg->device_id = sensor_->ParentName();
        msg->type = type;
        msg->status = TelemetryData::STATUS_OPERATIONAL;
        
        // Add sensor-specific data
        if (camera_) {
            msg->value_numeric = camera_->UpdateRate();
            msg->metadata = "{\"format\":\"" + camera_->ImageFormat() + "\"}";
        }

        telemetry_pub_->publish(std::move(*msg));
    }

    void publishDiagnostics() {
        auto now = node_->now();
        auto status = std::make_unique<diagnostic_msgs::msg::DiagnosticStatus>();
        status->name = sensor_->Name();
        status->hardware_id = sensor_->ParentName();

        if ((now - last_update_).seconds() > MAX_SENSOR_TIMEOUT) {
            status->level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status->message = "Sensor timeout";
        } else {
            status->level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status->message = "Operational";
        }

        diagnostic_pub_->publish(std::move(*status));
    }

    // Class members
    rclcpp::Node::SharedPtr node_;
    sensors::SensorPtr sensor_;
    sensors::CameraSensorPtr camera_;
    event::ConnectionPtr update_conn_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<TelemetryData>::SharedPtr telemetry_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_pub_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    std::unique_ptr<MessagePool<sensor_msgs::msg::Image>> image_pool_;
    rclcpp::Time last_update_;
};

// Register plugin with Gazebo
GZ_REGISTER_SENSOR_PLUGIN(SensorPlugin)

}  // namespace gazebo