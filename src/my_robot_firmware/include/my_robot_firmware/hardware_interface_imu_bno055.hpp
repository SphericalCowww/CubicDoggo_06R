#ifndef HARDWARE_INTERFACE_IMU_BNO055_HPP
#define HARDWARE_INTERFACE_IMU_BNO055_HPP

#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <gpiod.h>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace imu_namespace {
    class HardwareInterfaceBNO055_imu: public hardware_interface::SensorInterface
    {
        public:
            hardware_interface::CallbackReturn 
                on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
            hardware_interface::return_type 
                read(const rclcpp::Time & time, const rclcpp::Duration &period) override;
            hardware_interface::return_type 
                write(const rclcpp::Time & time, const rclcpp::Duration &period) override;
            
            hardware_interface::CallbackReturn 
                on_configure(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn 
                on_activate(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn 
                on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
            virtual ~HardwareInterfaceBNO055_imu();
        private:
            std::string i2c_bus_;
            uint8_t     address_;
            int         reset_pin_;
            std::string sensor_name_;

            struct gpiod_chip *gpio_chip_  = nullptr;
            struct gpiod_line *reset_line_ = nullptr;
            int    i2c_fd_                 = -1;

            double imu_states_[13] = {0.0}; 
    };    
}

#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
