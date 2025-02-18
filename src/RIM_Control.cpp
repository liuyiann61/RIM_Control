#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <wiringPi.h>
#include <thread>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>

class RIMNode : public rclcpp::Node
{
public:
    RIMNode() : Node("RIM_Control_Node")
    {
        // Initialize the subscription with correct message type
        RIM_Control = this->create_subscription<std_msgs::msg::Int32>(
            "/RIM_control", 
            10, 
            std::bind(&RIMNode::RIMControl, this, std::placeholders::_1)
        );
        
        // Setup WiringPi during construction
        wiringpi_setup();
    }

private:
    struct PWMInfo {
        unsigned int arr = 1000;  // auto-reload register
        unsigned int div = 120;   // divider
    };

    const int pwmpin=16;
    const int motorpin=18;
    const int em1pin=10;
    const int em2pin=12;
    PWMInfo motor_info;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr RIM_Control;

    void wiringpi_setup()
    {
        int model;
        piBoardId(&model);
        
        RCLCPP_INFO(this->get_logger(), "WiringPi setup starting...");
        
        wiringPiSetup();
        pinMode(pwmpin, PWM_OUTPUT);
        pinMode(motorpin, OUTPUT);
        pinMode(em1pin, OUTPUT);
        pinMode(em2pin, OUTPUT);
        RCLCPP_INFO(this->get_logger(), "WiringPi setup completed successfully");
    }

    void RIMControl(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int angle_data = msg->data;
        
        if (std::abs(angle_data) <= 30) 
        {
            single_rotation(angle_data);
        } 
        else
        {
            multiple_rotations(angle_data);
        }
    }

    void single_rotation(int angle)
    {
        pwmSetRange(pwmpin, motor_info.arr);
        pwmSetClock(pwmpin, motor_info.div);
        if (angle > 0 && angle <= 30)
        {
            digitalWrite(em1pin, HIGH);
            digitalWrite(em2pin, HIGH);
            digitalWrite(motorpin, HIGH);
            // Calculate CCR based on angle
            pwmWrite(pwmpin, 100);
            delay(std::abs(angle) * 55.5); 
            pwmWrite(pwmpin, 1000); // Delay proportional to angle
        }
        else if (angle < 0 && angle >= -30)
        {
            digitalWrite(em1pin, HIGH);
            digitalWrite(em2pin, HIGH);
            digitalWrite(motorpin, LOW);
            // Calculate CCR based on angle
            pwmWrite(pwmpin, 100);
            delay(std::abs(angle) * 55.5); 
            pwmWrite(pwmpin, 1000); // Delay proportional to angle
            digitalWrite(em1pin, LOW);
            digitalWrite(em2pin, LOW);
        }
    }
        
    void multiple_rotations(int angle)
    {
        pwmSetRange(pwmpin, motor_info.arr);
        pwmSetClock(pwmpin, motor_info.div);
        
        int num = std::abs(angle) / 30;
        int remain = angle % 30;
        
        if (angle > 0)
        {
            for (int i = 0; i < num; i++)
            {
                digitalWrite(em1pin, HIGH);
                digitalWrite(em2pin, HIGH);
                digitalWrite(motorpin, HIGH);
                // Calculate CCR based on angle
                pwmWrite(pwmpin, 100);
                delay(30 * 55.5); 
                digitalWrite(em1pin, LOW);
                digitalWrite(em2pin, LOW);
                digitalWrite(motorpin, LOW);
                pwmWrite(pwmpin, 100);
                delay(30 * 55.5); 
                pwmWrite(pwmpin, 1000); // Delay proportional to angle
            }
            if (remain != 0)
            {
                single_rotation(remain);
            }
        }
        else
        {
            for (int i = 0; i < num; i++)
            {
                digitalWrite(em1pin, HIGH);
                digitalWrite(em2pin, HIGH);
                digitalWrite(motorpin, LOW);
                // Calculate CCR based on angle
                pwmWrite(pwmpin, 100);
                delay(30 * 55.5); 
                digitalWrite(em1pin, LOW);
                digitalWrite(em2pin, LOW);
                digitalWrite(motorpin, HIGH);
                pwmWrite(pwmpin, 100);
                delay(30 * 55.5); 
                pwmWrite(pwmpin, 1000); // Delay proportional to angle
            }
            if (remain != 0)
            {
                single_rotation(remain);
            }
        }

    }
};
//5s rotate 90 degree,55.5ms/degree
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RIMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

