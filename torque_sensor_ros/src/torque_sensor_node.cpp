#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

// Include the 3rdparty driver header
// The include path will be configured in CMakeLists.txt
#include "torque_sensor/TorqueSensor.h"

using namespace std::chrono_literals;

namespace torque_sensor_ros {

class FrequencyCaculator {
public:
    FrequencyCaculator() {
        start_ = std::chrono::high_resolution_clock::now();
    }

    void sampleFrequency() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        start_ = end;
        if (duration.count() > 0) {
            frequency_hz_ = 1e6 / duration.count();
        }
    }

	double getFrequency() const {
		return frequency_hz_;
	}


	private:
    std::chrono::high_resolution_clock::time_point start_;
	double frequency_hz_ = 0.0;
};

class TorqueSensorNode : public rclcpp::Node {
public:
  TorqueSensorNode() : Node("torque_sensor_node") {
    // Declare parameters with default values
    this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 256000);
    this->declare_parameter<std::string>("frame_id", "torque_sensor_link");
    this->declare_parameter<double>("publish_rate", 1000.0); // Hz
    this->declare_parameter<std::string>("sensor_type", "RANGE_30NM"); // RANGE_30NM or RANGE_100NM
    this->declare_parameter<float>("zero_voltage", 5.0f);
    this->declare_parameter<float>("max_voltage", 10.0f);

    // Get parameters
    std::string port_name = this->get_parameter("port_name").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    std::string type_str = this->get_parameter("sensor_type").as_string();
    float zero_voltage = this->get_parameter("zero_voltage").as_double();
    float max_voltage = this->get_parameter("max_voltage").as_double();

    // Map string type to enum
    torque_sensor::TorqueSensor::TorqueSensorType type;
    if (type_str == "RANGE_100NM") {
      type = torque_sensor::TorqueSensor::TorqueSensorType::RANGE_100NM;
    } else {
      type = torque_sensor::TorqueSensor::TorqueSensorType::RANGE_30NM;
      if (type_str != "RANGE_30NM") {
        RCLCPP_WARN(this->get_logger(), "Unknown sensor type '%s', defaulting to RANGE_30NM", type_str.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Torque Sensor on %s at %d baud", port_name.c_str(), baudrate);
    RCLCPP_INFO(this->get_logger(), "Config: Type=%s, Zero=%.2fV, Max=%.2fV", 
                (type == torque_sensor::TorqueSensor::TorqueSensorType::RANGE_30NM ? "30Nm" : "100Nm"), 
                zero_voltage, max_voltage);

    // Initialize sensor driver
    try {
      sensor_ = std::make_unique<torque_sensor::TorqueSensor>(port_name, baudrate, type, zero_voltage, max_voltage);
      
      if (sensor_->connect()) {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to serial port.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to serial port %s", port_name.c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during initialization: %s", e.what());
    }

    // Publishers - Using ~/torque makes it private to the node's namespace
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("~/torque", rclcpp::SensorDataQoS());

    // Timer for polling and publishing
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(period, std::bind(&TorqueSensorNode::timer_callback, this));
  }

  ~TorqueSensorNode() {
    if (sensor_) {
      sensor_->disconnect();
    }
  }

private:
  FrequencyCaculator freqsampler;
  void timer_callback() {
    if (!sensor_) return;

    if (!sensor_->isConnected()) {
        static auto last_attempt = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (now - last_attempt > 2s) {
            RCLCPP_INFO(this->get_logger(), "Attempting to reconnect...");
            if (sensor_->connect()) {
                RCLCPP_INFO(this->get_logger(), "Reconnected!");
            }
            last_attempt = now;
        }
        return;
    }

    // Since the driver now uses a background thread, we just grab the latest torque
    // value and publish it. The timer dictates the publish rate.
    float torque = sensor_->getTorque();
    
    auto msg = geometry_msgs::msg::WrenchStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.wrench.torque.z = torque;
    // Publish frequency in force.x for diagnostics
    msg.wrench.force.x = freqsampler.getFrequency();
    msg.wrench.force.y = sensor_->getFrequency();
    
    wrench_pub_->publish(msg);
    
    // Sample frequency after successful publish
    freqsampler.sampleFrequency();
  }

  std::unique_ptr<torque_sensor::TorqueSensor> sensor_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
};

} // namespace torque_sensor_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<torque_sensor_ros::TorqueSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}