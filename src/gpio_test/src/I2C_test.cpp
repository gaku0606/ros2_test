#include <pigpiod_if2.h> //pigpio.hは実行時にrootユーザーになる必要があるためこっちを使用

//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

#include "mpu9250_i2c.h"
//#include "I2C.hpp"

/********************************************
 *  I2C通信を試すプログラム
 *  9軸センサを動かしてみる
 * 参考: http://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_open
 * 2022/08/07 Gaku
*********************************************/

int main(int argc, char* argv[]){
    //int pi;
    //pi = pigpio_start(NULL, NULL);

    using namespace::std::chrono_literals;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gpio");
    auto publisher = node->create_publisher<std_msgs::msg::String>("gpio", 10);

    std_msgs::msg::String str;
    str.data = "hello world";

    rclcpp::WallRate rate(50ms);

    //i2cBus(I2C_0かI2C_1の選択、GPIO2と3はI2C_1), i2cAddr(7bitスレーブアドレス), i2cFlags(0に設定する)
    //I2C i2c(MPU9250_SLAVE_ADDR_HIGH, 1);
    mpu9250 nine(AD0_HIGH);
    //int imu_i2c = i2c_open(pi, 1, MPU9250_SLAVE_ADDR_HIGH, 0);    
    if(nine.sensorTest()){//imu_i2c >= 0){
        std::cout << "IMU-sensor connection OK!!" << std::endl;
    }
    else{
        std::cout << "IMU-sensor connection faild" << std::endl;
    }

    if(nine.mag_sensorTest())
        std::cout << "Mag-sensor OK" << std::endl;
    else    
        std::cout << "Mag-sensor NG" << std::endl;

    nine.setOffset( 0.417502, 2.048989, -0.04437,
                    -0.00588,0.03913,0.03507,
                    26.4, 4.425, 32.25);

    //int read_data = i2c_read_byte_data(pi, imu_i2c, WHO_AM_I_MPU9250);
    // char read_data = 0;
    // i2c.read(WHO_AM_I_MPU9250, &read_data, 1);
    // printf("WHO_AM_I: %0X\r\n", read_data);
    // if(read_data == 0x71)
    //     std::cout << "connection OK!!" << std::endl;  

    double nine_data[9] = {};

    double mag_max[3] = {}, mag_min[3] = {}, mag_ave[3] = {};

    //メインループ
    while(rclcpp::ok()){//Ctrl+Cで脱出
        nine.getAll(nine_data);
        char s[128];
        // sprintf(s, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f"
        // , nine_data[0],nine_data[1],nine_data[2]
        // , nine_data[3],nine_data[4],nine_data[5]
        // , nine_data[6],nine_data[7],nine_data[8]);

        if(nine_data[6] > mag_max[0]) mag_max[0] = nine_data[6];
        else if(nine_data[6] < mag_min[0]) mag_min[0] = nine_data[6];

        if(nine_data[7] > mag_max[1]) mag_max[1] = nine_data[7];
        else if(nine_data[7] < mag_min[1]) mag_min[1] = nine_data[7];
        
        if(nine_data[8] > mag_max[2]) mag_max[2] = nine_data[8];
        else if(nine_data[8] < mag_min[2]) mag_min[2] = nine_data[8];
        
        mag_ave[0] = (mag_max[0] + mag_min[0]) / 2.0;
        mag_ave[1] = (mag_max[1] + mag_min[1]) / 2.0;
        mag_ave[2] = (mag_max[2] + mag_min[2]) / 2.0;

        sprintf(s, "%.4f\t%.4f\t%.4f"
//        , nine_data[0],nine_data[1],nine_data[2]
//      , nine_data[3],nine_data[4],nine_data[5]
        //, nine_data[6],nine_data[7],nine_data[8]
        , mag_ave[0], mag_ave[1], mag_ave[2]
        );

        printf("%s\n", s);
        //RCLCPP_INFO(node->get_logger(), "Publishing gpio '%s'", str.data.c_str());
        publisher->publish(str);
        rclcpp::spin_some(node);

        rate.sleep();
    }
    rclcpp::shutdown();

    //i2c_close(pi, imu_i2c);

    return 0;
}
