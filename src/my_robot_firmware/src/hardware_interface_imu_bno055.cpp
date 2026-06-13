#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_firmware/hardware_interface_imu_bno055.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace imu_namespace {
    hardware_interface::CallbackReturn HardwareInterfaceBNO055_imu::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) 
    {
        if (hardware_interface::SensorInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_logger(), "hardware_interface_imu:on_init()");

        /////////// see: src/my_robot_description/urdf/cubic_doggo.ros2_control.xacro
        try {
            i2c_bus_     = params.hardware_info.hardware_parameters.at("i2c_bus");
            address_     = std::stoi(params.hardware_info.hardware_parameters.at("address"), nullptr, 16);
            reset_pin_   = std::stoi(params.hardware_info.hardware_parameters.at("reset_pin"));
            sensor_name_ = params.hardware_info.sensors[0].name;
        } catch (const std::out_of_range& errorMsg) {
            RCLCPP_ERROR(get_logger(), "hardware_interface_imu:on_init(): missing required parameter in URDF");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_logger(), "hardware_interface_imu:on_init(): parameters loaded");
        ///////////

        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::CallbackReturn HardwareInterfaceBNO055_imu::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)  
    {
        RCLCPP_INFO(get_logger(), "hardware_interface_imu:on_configure()");
        
        gpio_chip_ = gpiod_chip_open_by_name("gpiochip4");
        if (!gpio_chip_) {
            RCLCPP_ERROR(get_logger(), "hardware_interface_imu:on_configure(): "
                                       "failed to open gpiochip4 (Pi 5 header)");
            return CallbackReturn::ERROR;
        }
 
        reset_line_ = gpiod_chip_get_line(gpio_chip_, reset_pin_);
        if (!reset_line_) {
            RCLCPP_ERROR(get_logger(), "hardware_interface_imu:on_configure(): "
                                       "failed to get reset line %d", reset_pin_);
            return CallbackReturn::ERROR;
        }
        gpiod_line_request_output(reset_line_, "bno_reset", 1);
        gpiod_line_set_value(reset_line_, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        gpiod_line_set_value(reset_line_, 1); 
        std::this_thread::sleep_for(std::chrono::milliseconds(700)); // wait for boot

        i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);
        if (i2c_fd_ < 0 || ioctl(i2c_fd_, I2C_SLAVE, address_) < 0) {
            RCLCPP_ERROR(get_logger(), "hardware_interface_imu:on_configure(): "
                                       "failed to open I2C bus or connect to 0x%02x", address_);
            return CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceBNO055_imu::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)  
    {
        RCLCPP_INFO(get_logger(), "hardware_interface_imu:on_activate()");
       
        uint8_t config[2] = {0x3D, 0x0C}; 
        if (::write(i2c_fd_, config, 2) != 2) {
            RCLCPP_ERROR(get_logger(), "hardware_interface_imu:on_activate(): failed to set NDOF mode");
            return CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceBNO055_imu::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)  
    {
        RCLCPP_INFO(get_logger(), "hardware_interface_imu:on_deactivate()");

        uint8_t config[2] = {0x3D, 0x00};
        ::write(i2c_fd_, config, 2);

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    HardwareInterfaceBNO055_imu::~HardwareInterfaceBNO055_imu() {
        if (i2c_fd_ >= 0) ::close(i2c_fd_);
        if (reset_line_)  gpiod_line_release(reset_line_);
        if (gpio_chip_)   gpiod_chip_close(gpio_chip_);
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::return_type HardwareInterfaceBNO055_imu::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        RCLCPP_DEBUG(get_logger(), "hardware_interface_imu:read()");

        uint8_t reg = 0x08;
        uint8_t data[44];

        if ((::write(i2c_fd_, &reg, 1) != 1) || (::read(i2c_fd_, data, 44) != 44)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "hardware_interface_imu:read():"
                                                                   "I2C Burst Read Failed");
            return hardware_interface::return_type::ERROR;
        }

        // linear acceleration in m/s^2
        imu_states_[0] = static_cast<int16_t>(data[33] << 8 | data[32]) / 100.0; // linear_acceleration.x
        imu_states_[1] = static_cast<int16_t>(data[35] << 8 | data[34]) / 100.0; // linear_acceleration.y
        imu_states_[2] = static_cast<int16_t>(data[37] << 8 | data[36]) / 100.0; // linear_acceleration.z
 
        // angular velocity in rad/s
        double dps_to_rads = M_PI / 180.0;
        imu_states_[3] = (static_cast<int16_t>(data[13] << 8 | data[12]) / 16.0) * dps_to_rads; // angular_velocity.x
        imu_states_[4] = (static_cast<int16_t>(data[15] << 8 | data[14]) / 16.0) * dps_to_rads; // angular_velocity.y
        imu_states_[5] = (static_cast<int16_t>(data[17] << 8 | data[16]) / 16.0) * dps_to_rads; // angular_velocity.z
    
        // orientation quaternion  
        double q_scale = 1.0 / 16384.0; // 2^14
        imu_states_[6] = static_cast<int16_t>(data[25] << 8 | data[24]) * q_scale; // orientation.w
        imu_states_[7] = static_cast<int16_t>(data[27] << 8 | data[26]) * q_scale; // orientation.x
        imu_states_[8] = static_cast<int16_t>(data[29] << 8 | data[28]) * q_scale; // orientation.y
        imu_states_[9] = static_cast<int16_t>(data[31] << 8 | data[30]) * q_scale; // orientation.z

        // euler angles (reg 0x1A-0x1F) 
        imu_states_[10] = static_cast<int16_t>(data[21] << 8 | data[20]) / 16.0; // roll
        imu_states_[11] = static_cast<int16_t>(data[23] << 8 | data[22]) / 16.0; // pitch
        imu_states_[12] = static_cast<int16_t>(data[19] << 8 | data[18]) / 16.0; // heading

        set_state(sensor_name_ + "/linear_acceleration.x", imu_states_[0]);
        set_state(sensor_name_ + "/linear_acceleration.y", imu_states_[1]);
        set_state(sensor_name_ + "/linear_acceleration.z", imu_states_[2]);
        set_state(sensor_name_ + "/angular_velocity.x",    imu_states_[3]);
        set_state(sensor_name_ + "/angular_velocity.y",    imu_states_[4]);
        set_state(sensor_name_ + "/angular_velocity.z",    imu_states_[5]);
        set_state(sensor_name_ + "/orientation.w",         imu_states_[6]);
        set_state(sensor_name_ + "/orientation.x",         imu_states_[7]);
        set_state(sensor_name_ + "/orientation.y",         imu_states_[8]);
        set_state(sensor_name_ + "/orientation.z",         imu_states_[9]);
        set_state(sensor_name_ + "/euler.x",               imu_states_[10]);
        set_state(sensor_name_ + "/euler.y",               imu_states_[11]);
        set_state(sensor_name_ + "/euler.z",               imu_states_[12]);

        return hardware_interface::return_type::OK;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(imu_namespace::HardwareInterfaceBNO055_imu, hardware_interface::SensorInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









