#ifndef _I2C_HPP_ 
#define _I2C_HPP_

/****************************************
* pigpioのI2Cをmbedと同じ書き方にする
*
*****************************************/
#include <pigpiod_if2.h>
#include <iostream>
#include <stdio.h>

class I2C{
    int pi;
    int i2c;

public:
    I2C(char slave_addr, int i2cBus = 1){
        pi = pigpio_start(NULL, NULL);
        if(i2cBus < 0) i2cBus = 1;
        i2c = i2c_open(pi, i2cBus, slave_addr, 0);
        // if(i2c >= 0)
        //     std::cout << "I2C connection OK" << std::endl;
        // else    
        //     std::cout << "I2C connection Error" << std::endl;
        //int read_data = i2c_read_byte_data(pi, imu_i2c, WHO_AM_I_MPU9250);
        // char read_data = read(0x75);//, &read_data, 1);
        // printf("WHO_AM_I: %0X\r\n", read_data);
        // if(read_data == 0x71)
        //     std::cout << "connection OK!!" << std::endl;  
    }

    ~I2C(){
        i2c_close(pi, i2c);
        pigpio_stop(pi);
    }


    //write 1 byte
    void write(unsigned char reg_addr, unsigned char data_array){
        i2c_write_byte_data(pi, i2c, reg_addr, data_array);
    }

    //write N-byte
    void write(unsigned char addr, char* data_array, unsigned int count){
        i2c_write_block_data(pi, i2c, addr, data_array, count);
    }

    //read 1 byte
    char read(unsigned char reg_addr){
        return i2c_read_byte_data(pi, i2c, reg_addr);
    }

    //read N-byte
    void read(unsigned char reg_addr, char* buffer, unsigned int count){
        i2c_read_i2c_block_data(pi, i2c, reg_addr, buffer, count);
    }

    int getHandle(){
        return i2c;
    }

};

#endif