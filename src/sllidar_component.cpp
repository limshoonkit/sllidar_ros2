#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include "sl_lidar.h"
#include "math.h"

#include <chrono>
#include <memory>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#define ROS2VERSION "1.0.1"

namespace sllidar_ros2
{

using namespace sl;

class SLlidarComponent : public rclcpp::Node
{
  public:
    explicit SLlidarComponent(const rclcpp::NodeOptions & options)
    : Node("sllidar_node", options)
    {
        // --- Initialize parameters
        this->declare_parameter<std::string>("channel_type","serial");
        this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");
        this->declare_parameter<int>("tcp_port", 20108);
        this->declare_parameter<std::string>("udp_ip","192.168.11.2");
        this->declare_parameter<int>("udp_port",8089);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate",1000000);
        this->declare_parameter<std::string>("frame_id","laser_frame");
        this->declare_parameter<bool>("inverted", false);
        this->declare_parameter<bool>("angle_compensate", false);
        this->declare_parameter<std::string>("scan_mode",std::string());
        this->declare_parameter<float>("scan_frequency",10.0f);

        this->get_parameter("channel_type", channel_type);
        this->get_parameter("tcp_ip", tcp_ip);
        this->get_parameter("tcp_port", tcp_port);
        this->get_parameter("udp_ip", udp_ip);
        this->get_parameter("udp_port", udp_port);
        this->get_parameter("serial_port", serial_port);
        this->get_parameter("serial_baudrate", serial_baudrate);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("inverted", inverted);
        this->get_parameter("angle_compensate", angle_compensate);
        this->get_parameter("scan_mode", scan_mode);
        this->get_parameter("scan_frequency", scan_frequency);
        
        RCLCPP_INFO(this->get_logger(),"SLLidar running on ROS2 package SLLidar.ROS2 SDK Version: %s, SLLIDAR SDK Version:%d.%d.%d",
                ROS2VERSION, SL_LIDAR_SDK_VERSION_MAJOR, SL_LIDAR_SDK_VERSION_MINOR, SL_LIDAR_SDK_VERSION_PATCH);

        // --- Create publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));

        // --- Create LIDAR driver
        drv_ = std::unique_ptr<ILidarDriver>(*createLidarDriver());
        if (!drv_) {
            throw std::runtime_error("Failed to create LIDAR driver.");
        }

        // --- Create and connect channel
        IChannel* channel = nullptr;
        if (channel_type == "tcp") {
            channel = *createTcpChannel(tcp_ip, tcp_port);
        } else if (channel_type == "udp") {
            channel = *createUdpChannel(udp_ip, udp_port);
        } else {
            channel = *createSerialPortChannel(serial_port, serial_baudrate);
        }

        if (SL_IS_FAIL(drv_->connect(channel))) {
            RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to the specified device. port: %s, baudrate: %d, channel: %s",
                serial_port.c_str(), serial_baudrate, channel_type.c_str());
            throw std::runtime_error("Failed to connect to LIDAR.");
        }

        // --- Get device info and health
        if (!getSLLIDARDeviceInfo(drv_.get())) {
            throw std::runtime_error("Failed to get device info.");
        }
        if (!checkSLLIDARHealth(drv_.get())) {
            throw std::runtime_error("LIDAR health check failed.");
        }

        // --- Create services
        stop_motor_service_ = this->create_service<std_srvs::srv::Empty>("stop_motor",
                std::bind(&SLlidarComponent::stop_motor, this, std::placeholders::_1, std::placeholders::_2));
        start_motor_service_ = this->create_service<std_srvs::srv::Empty>("start_motor",
                std::bind(&SLlidarComponent::start_motor, this, std::placeholders::_1, std::placeholders::_2));

        // --- Start motor and scan
        drv_->setMotorSpeed();

        LidarScanMode current_scan_mode;
        sl_result op_result = drv_->startScan(false, true, 0, &current_scan_mode);

        if (SL_IS_OK(op_result)) {
            int points_per_circle = (int)(1000*1000/current_scan_mode.us_per_sample/scan_frequency);
            angle_compensate_multiple_ = points_per_circle/360.0  + 1;
            if (angle_compensate_multiple_ < 1) angle_compensate_multiple_ = 1.0;
            max_distance_ = (float)current_scan_mode.max_distance;
            RCLCPP_INFO(this->get_logger(),"Current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz",
                        current_scan_mode.scan_mode,(int)(1000/current_scan_mode.us_per_sample+0.5),max_distance_, scan_frequency);
        } else {
            RCLCPP_ERROR(this->get_logger(),"Cannot start scan: %08x!", op_result);
            throw std::runtime_error("Failed to start LIDAR scan.");
        }

        // --- Create a timer to poll for scan data
        auto timer_period = std::chrono::duration<double>(1.0 / scan_frequency);
        timer_ = this->create_wall_timer(timer_period, std::bind(&SLlidarComponent::scan_loop_callback, this));
    }

    ~SLlidarComponent()
    {
        if (drv_) {
            drv_->setMotorSpeed(0);
            drv_->stop();
        }
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string channel_type;
    std::string tcp_ip;
    std::string udp_ip;
    std::string serial_port;
    int tcp_port;
    int udp_port;
    int serial_baudrate;
    std::string frame_id;
    bool inverted;
    bool angle_compensate;
    float max_distance_;
    size_t angle_compensate_multiple_;
    std::string scan_mode;
    float scan_frequency;

    std::unique_ptr<ILidarDriver> drv_;

    void scan_loop_callback()
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        rclcpp::Time start_scan_time = this->now();
        sl_result op_result = drv_->grabScanDataHq(nodes, count);
        rclcpp::Time end_scan_time = this->now();
        double scan_duration = (end_scan_time - start_scan_time).seconds();

        if (op_result == SL_RESULT_OK) {
            op_result = drv_->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(360.0f);
            if (op_result == SL_RESULT_OK) {
                if (angle_compensate) {
                    const size_t angle_compensate_nodes_count = 360 * angle_compensate_multiple_;
                    auto angle_compensate_nodes = std::make_unique<sl_lidar_response_measurement_node_hq_t[]>(angle_compensate_nodes_count);
                    
                    for (size_t i = 0; i < count; i++) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple_);
                            if (angle_value >= 0 && (size_t)angle_value < angle_compensate_nodes_count) {
                                angle_compensate_nodes[angle_value] = nodes[i];
                            }
                        }
                    }
                    publish_scan(angle_compensate_nodes.get(), angle_compensate_nodes_count, start_scan_time, scan_duration, inverted, angle_min, angle_max, max_distance_, frame_id);
                } else {
                    size_t start_node = 0, end_node = 0;
                    // Fix signed/unsigned comparison warning by using size_t for loop variable
                    for (size_t i = 0; i < count; ++i) {
                        if (nodes[i].dist_mm_q2 != 0) {
                           start_node = i;
                           break;
                        }
                    }
                    for (size_t i = count; i > 0; --i) {
                        if (nodes[i-1].dist_mm_q2 != 0) {
                           end_node = i-1;
                           break;
                        }
                    }

                    if (start_node < end_node) {
                        angle_min = DEG2RAD(getAngle(nodes[start_node]));
                        angle_max = DEG2RAD(getAngle(nodes[end_node]));
                        publish_scan(&nodes[start_node], end_node - start_node + 1, start_scan_time, scan_duration, inverted, angle_min, angle_max, max_distance_, frame_id);
                    }
                }
            } else if (op_result == SL_RESULT_OPERATION_FAIL) {
                publish_scan(nodes, count, start_scan_time, scan_duration, inverted, angle_min, angle_max, max_distance_, frame_id);
            }
        }
    }

    bool getSLLIDARDeviceInfo(ILidarDriver * drv)
    {
        sl_lidar_response_device_info_t devinfo;
        sl_result op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            RCLCPP_ERROR(this->get_logger(), "Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!");
            return false;
        }
        char sn_str[37] = {'\0'};
        for (int pos = 0; pos < 16; ++pos) {
            sprintf(sn_str + (pos * 2), "%02X", devinfo.serialnum[pos]);
        }
        RCLCPP_INFO(this->get_logger(), "SLLidar S/N: %s", sn_str);
        RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d", (int)devinfo.hardware_version);
        return true;
    }

    bool checkSLLIDARHealth(ILidarDriver * drv)
    {
        sl_lidar_response_device_health_t healthinfo;
        sl_result op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) {
            RCLCPP_INFO(this->get_logger(), "SLLidar health status : %s",
                healthinfo.status == SL_LIDAR_STATUS_OK ? "OK" :
                (healthinfo.status == SL_LIDAR_STATUS_WARNING ? "Warning" : "Error"));
            if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
                RCLCPP_ERROR(this->get_logger(), "Error, SLLidar internal error detected. Please reboot the device to retry.");
                return false;
            }
            return true;
        }
        RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve SLLidar health code: %x", op_result);
        return false;
    }

    bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        if (!drv_) return false;
        RCLCPP_DEBUG(this->get_logger(), "Stop motor");
        drv_->setMotorSpeed(0);
        return true;
    }

    bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        if (!drv_) return false;
        if (drv_->isConnected()) {
            RCLCPP_DEBUG(this->get_logger(), "Start motor");
            drv_->setMotorSpeed();
            drv_->startScan(0, 1);
        } else {
            RCLCPP_INFO(this->get_logger(), "Lost connection");
            return false;
        }
        return true;
    }

    static float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    void publish_scan(sl_lidar_response_measurement_node_hq_t *nodes, size_t node_count, rclcpp::Time start,
                      double scan_time, bool inverted, float angle_min, float angle_max, float max_distance, std::string frame_id)
    {
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id;

        bool reversed = (angle_max > angle_min);
        scan_msg->angle_min = reversed ? (M_PI - angle_max) : (M_PI - angle_min);
        scan_msg->angle_max = reversed ? (M_PI - angle_min) : (M_PI - angle_max);
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count > 1 ? node_count - 1 : 1);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(node_count > 1 ? node_count - 1 : 1);
        scan_msg->range_min = 0.05;
        scan_msg->range_max = max_distance;

        scan_msg->intensities.resize(node_count);
        scan_msg->ranges.resize(node_count);
        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);

        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000.0f;
            size_t index = reverse_data ? (node_count - 1 - i) : i;

            if (read_value == 0.0) {
                scan_msg->ranges[index] = std::numeric_limits<float>::infinity();
            } else {
                scan_msg->ranges[index] = read_value;
            }
            scan_msg->intensities[index] = (float)(nodes[i].quality >> 2);
        }
        scan_pub_->publish(std::move(scan_msg));
    }
};

} // namespace sllidar_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sllidar_ros2::SLlidarComponent)