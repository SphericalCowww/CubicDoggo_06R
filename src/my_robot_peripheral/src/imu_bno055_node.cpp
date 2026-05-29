#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <chrono>
#include <filesystem> 
#include <thread>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IMUBNO055Node : public rclcpp::Node {
public:
    IMUBNO055Node() : Node("bno055_node") {
        this->declare_parameter("i2c_bus",   "/dev/i2c-1");
        this->declare_parameter("address",   0x28);
        this->declare_parameter("reset_pin", "17");

        if (setup_hardware() == true) {
            RCLCPP_INFO(get_logger(), "IMUBNO055Node(): Connection established and sensor initialized.");
            imu_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu/euler", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), 
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
        std::string i2c_bus   = this->get_parameter("i2c_bus").as_string();
        int address           = this->get_parameter("address").as_int();
        std::string reset_pin = this->get_parameter("reset_pin").as_string();

        auto gpio_write = [&](const std::string& path, const std::string& val) {
            std::ofstream write_file(path);
            if (write_file.is_open()) {
                write_file << val;
            }
        };

        RCLCPP_INFO(get_logger(), "IMUBNO055Node():setup_hardware(): " 
                                  "Resetting sensor on GPIO %s...", reset_pin.c_str());
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
            RCLCPP_ERROR(get_logger(), "IMUBNO055Node():setup_hardware(): Failed to open I2C bus %s",i2c_bus.c_str());
            return false;
        }
        if (ioctl(i2c_fd_, I2C_SLAVE, address) < 0) {
            RCLCPP_ERROR(get_logger(), "IMUBNO055Node():setup_hardware(): "
                                       "Failed to connect to BNO055 at 0x%02x", address);
            return false;
        }

        uint8_t config[2] = {0x3D, 0x0C};
        if (write(i2c_fd_, config, 2) != 2) {
            RCLCPP_ERROR(get_logger(), "IMUBNO055Node():setup_hardware(): Failed to set NDOF mode");
            return false;
        }
        return true;
    }

    void read_and_publish() {
        uint8_t reg = 0x1A; // Euler data starts at 0x1A
        uint8_t data[6];    // order: Heading LSB, Heading MSB, Roll LSB, Roll MSB, Pitch LSB, Pitch MSB

        if (write(i2c_fd_, &reg, 1) != 1) return;
        if (read(i2c_fd_, data, 6) != 6) return;

        // convert to degrees (1 degree = 16 LSB)
        int16_t h_raw = (data[1] << 8) | data[0];
        int16_t r_raw = (data[3] << 8) | data[2];
        int16_t p_raw = (data[5] << 8) | data[4];

        auto imu_msg = geometry_msgs::msg::Vector3();
        imu_msg.z = static_cast<double>(h_raw) / 16.0; // Heading
        imu_msg.x = static_cast<double>(r_raw) / 16.0; // Roll
        imu_msg.y = static_cast<double>(p_raw) / 16.0; // Pitch

        imu_pub_->publish(imu_msg);
    }

    int i2c_fd_ = -1;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_pub_;
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



