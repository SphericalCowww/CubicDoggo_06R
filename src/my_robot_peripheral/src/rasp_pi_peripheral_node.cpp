#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <fstream>
#include <filesystem>
#include <string>
#include <chrono>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RapsPiPeripheralNode : public rclcpp::Node {
public:
    RapsPiPeripheralNode() : Node("pi_peripheral_node") {
        this->declare_parameter("alarm_path", "/sys/class/hwmon/hwmon3/in0_lcrit_alarm");
        this->declare_parameter("volt_path",  "/sys/class/hwmon/hwmon3/in0_input");
        this->declare_parameter("led_path",   "/sys/class/leds/ACT/");
        this->declare_parameter("delay_ms",   500);

        std::string alarm_path = this->get_parameter("alarm_path").as_string();
        std::string volt_path  = this->get_parameter("volt_path") .as_string();
        std::string led_path   = this->get_parameter("led_path")  .as_string();
        is_rasp_pi_ = std::filesystem::exists(alarm_path) && 
                      std::filesystem::exists(volt_path)  &&
                      std::filesystem::exists(led_path);
        if (is_valid() == true) {
            RCLCPP_INFO(get_logger(), "RapsPiPeripheralNode:constructor(): raspberry pi "
                                      "detecting power alarm at %s, power volt at %s, led detected at %s", 
                                      alarm_path.c_str(), volt_path.c_str(), led_path.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "RapsPiPeripheralNode:constructor(): not on a raspberry pi (or no LED access)");
        }
        volt_pub_ = this->create_publisher<example_interfaces::msg::Float64>("pi/voltage", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&RapsPiPeripheralNode::checkHardwarePower, this));
    }
    bool is_valid() const { 
        return is_rasp_pi_; 
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    void checkHardwarePower() {
        if (is_rasp_pi_ == false) return;

        std::string alarm_path = this->get_parameter("alarm_path").as_string();
        std::ifstream alarm_file(alarm_path);
        if (alarm_file.is_open()) {
            int power_alarm;
            alarm_file >> power_alarm;
            if (power_alarm == 1) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(),  5000, "RapsPiPeripheralNode:checkHardwarePower(): "
                                      "HARDWARE LOW POWER, TIME TO CHARGE BATTERY");
                setLedTrigger("timer");
            } else {
                RCLCPP_INFO(get_logger(), "RapsPiPeripheralNode:checkHardwarePower(): hardware power normal");
                setLedTrigger("none");
            }
        }

        std::string volt_path = this->get_parameter("volt_path").as_string();
        std::ifstream volt_file(volt_path);
        if (volt_file.is_open()) {
            double volt_mv;
            volt_file >> volt_mv;
            
            auto volt_msg = example_interfaces::msg::Float64();
            volt_msg.data = volt_mv / 1000.0;                        // to volt
            volt_pub_->publish(volt_msg);
        }
    }
    void setLedTrigger(const std::string &trigger) {
        if (current_trigger_ == trigger) return;

        std::string led_path = this->get_parameter("led_path").as_string();
        int delay_ms = this->get_parameter("delay_ms").as_int();
        
        std::ofstream led_trigger_file(led_path+"/trigger");
        if (led_trigger_file.is_open()) {
            led_trigger_file << trigger;
            current_trigger_ = trigger;
            RCLCPP_INFO(get_logger(), "RapsPiPeripheralNode:setLedTrigger(): "
                                      "LED trigger set to: %s", trigger.c_str());
        }
        if (trigger == "timer") {
            std::ofstream led_on_file( led_path+"/delay_on");
            std::ofstream led_off_file(led_path+"/delay_off");
            if (led_on_file.is_open() && led_off_file.is_open()) {
                led_on_file << delay_ms;
                led_off_file << delay_ms;
            }
        }
    }

    bool        is_rasp_pi_ = false;
    std::string current_trigger_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr volt_pub_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RapsPiPeripheralNode>();
    if (node->is_valid()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
