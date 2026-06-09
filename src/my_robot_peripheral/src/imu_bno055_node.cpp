#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <chrono>
#include <filesystem> 
#include <thread>
#include <cmath> 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IMUBNO055Node : public rclcpp::Node {
public:
    IMUBNO055Node() : Node("bno055_node") {
        this->declare_parameter("i2c_bus",   "/dev/i2c-1");
        this->declare_parameter("address",   0x28);
        this->declare_parameter("reset_pin", "17");

        if (setup_hardware() == true) {
            RCLCPP_INFO(get_logger(), "IMUBNO055Node(): Connection established and sensor initialized.");
            imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>(      "imu/data",  10);
            euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu/euler", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),                                                          // 100 Hz 
                std::bind(&IMUBNO055Node::read_and_publish, this)
            );
        }
    }
    ~IMUBNO055Node() {
        if (i2c_fd_ >= 0) close(i2c_fd_);
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    bool setup_hardware() {
        std::string i2c_bus   = this->get_parameter("i2c_bus")  .as_string();
        int address           = this->get_parameter("address")  .as_int();
        std::string reset_pin = this->get_parameter("reset_pin").as_string();

        auto gpio_write = [&](const std::string& path, const std::string& val) {
            std::ofstream write_file(path);
            if (write_file.is_open()) {
                write_file << val;
            }
        };

        RCLCPP_INFO(get_logger(), "IMUBNO055Node():setup_hardware(): " 
                                  "resetting sensor on GPIO %s...", reset_pin.c_str());
        std::string gpio_base = "/sys/class/gpio/gpio" + reset_pin;
        if (std::filesystem::exists(gpio_base) ==  false) {
            std::ofstream export_file("/sys/class/gpio/export");
            if (export_file.is_open()) {
                export_file << reset_pin;
                export_file.close();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        
        gpio_write(gpio_base + "/direction", "out");
        gpio_write(gpio_base + "/value", "0");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        gpio_write(gpio_base + "/value", "1");
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        i2c_fd_ = open(i2c_bus.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "IMUBNO055Node():setup_hardware(): failed to open I2C bus %s",i2c_bus.c_str());
            return false;
        }
        if (ioctl(i2c_fd_, I2C_SLAVE, address) < 0) {
            RCLCPP_ERROR(get_logger(), "IMUBNO055Node():setup_hardware(): "
                                       "Failed to connect to BNO055 at 0x%02x", address);
            return false;
        }

        uint8_t config[2] = {0x3D, 0x0C};
        if (write(i2c_fd_, config, 2) != 2) {
            RCLCPP_ERROR(get_logger(), "IMUBNO055Node():setup_hardware(): failed to set NDOF mode");
            return false;
        }
        return true;
    }

    void read_and_publish() {
        uint8_t reg = 0x08;
        uint8_t data[44]; 

        if (write(i2c_fd_, &reg, 1) != 1) return;
        if (read(i2c_fd_, data, 44) != 44) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "I2C Burst Read Failed");
            return;
        }

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        // linear acceleration in m/s^2
        imu_msg.linear_acceleration.x = static_cast<int16_t>(data[33] << 8 | data[32]) / 100.0;
        imu_msg.linear_acceleration.y = static_cast<int16_t>(data[35] << 8 | data[34]) / 100.0;
        imu_msg.linear_acceleration.z = static_cast<int16_t>(data[37] << 8 | data[36]) / 100.0;

        // angular velocity in rad/s
        double dps_to_rads = M_PI / 180.0;
        imu_msg.angular_velocity.x = (static_cast<int16_t>(data[13] << 8 | data[12]) / 16.0) * dps_to_rads;
        imu_msg.angular_velocity.y = (static_cast<int16_t>(data[15] << 8 | data[14]) / 16.0) * dps_to_rads;
        imu_msg.angular_velocity.z = (static_cast<int16_t>(data[17] << 8 | data[16]) / 16.0) * dps_to_rads;

        // orientation quaternion 
        double q_scale = 1.0 / 16384.0; // 2^14
        imu_msg.orientation.w = static_cast<int16_t>(data[25] << 8 | data[24]) * q_scale;
        imu_msg.orientation.x = static_cast<int16_t>(data[27] << 8 | data[26]) * q_scale;
        imu_msg.orientation.y = static_cast<int16_t>(data[29] << 8 | data[28]) * q_scale;
        imu_msg.orientation.z = static_cast<int16_t>(data[31] << 8 | data[30]) * q_scale;

        // euler angles (reg 0x1A-0x1F) 
        auto euler_msg = geometry_msgs::msg::Vector3();
        euler_msg.x = static_cast<int16_t>(data[21] << 8 | data[20]) / 16.0; // Roll
        euler_msg.y = static_cast<int16_t>(data[23] << 8 | data[22]) / 16.0; // Pitch
        euler_msg.z = static_cast<int16_t>(data[19] << 8 | data[18]) / 16.0; // Heading

        imu_pub_->publish(imu_msg);
        euler_pub_->publish(euler_msg);
    }

    int i2c_fd_ = -1;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr       imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_;
    rclcpp::TimerBase::SharedPtr                              timer_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUBNO055Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



