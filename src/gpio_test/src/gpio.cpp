// #include <pigpio.h>
#include <pigpiod_if2.h>
// #include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

#define LED_PIN 18

int main(int argc, char* argv[]){

    using namespace::std::chrono_literals;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gpio");
    auto publisher = node->create_publisher<std_msgs::msg::String>("gpio", 10);

    std_msgs::msg::String str;
    str.data = "hello world";

    rclcpp::WallRate rate(200ms);

    int pi = pigpio_start(NULL, NULL);

    // gpioSetMode(LED_PIN, PI_OUTPUT);
    int led = 0;

    while(rclcpp::ok()){
        RCLCPP_INFO(node->get_logger(), "Publishing gpio '%s'", str.data.c_str());
        publisher->publish(str);
        rclcpp::spin_some(node);
        
        if(led == 0) led = 1;
        else led = 0;

        gpio_write(pi, LED_PIN, led);//gpioWrite(LED_PIN, led);
        rate.sleep();
    }
    rclcpp::shutdown();

    return 0;
}