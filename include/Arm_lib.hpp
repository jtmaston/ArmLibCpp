#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "./Arm_exc.hpp"

#include <chrono>
#include <thread>

#include <cmath>
#include <arm_neon.h>
#include <array>
#include <vector>
#include <linux/joystick.h>

extern "C"
{
#include "linux/i2c-dev.h"
#include <i2c/smbus.h>
}

#define BLOCK_SIZE 4
#define __PI__ 3.14159265359
#define __RAD__ 0.0174533

class ArmDevice
{
    public:
        float angles[7] = {0, 0, 0, 0, 0, 0, 0};
        std::vector<std::vector<float32_t>> learned_angles;

        ArmDevice();


        void buzz(uint8_t time = 10);                                                   // start the buzzer
        void noBuzz();                                                                  // stop the buzzer

        void rgb(uint8_t r, uint8_t g, uint8_t b);                                      // set the RGB led value
        void reset_mcu();                                                               // reset microcontroller
        bool ping_servo(uint8_t id);                                                    // ping to test if servo is available
        void button_mode(bool mode);                                                    // undocumented function

        void servo_write_any(uint8_t id, uint16_t angle, uint16_t time);                // write value to any servo ( id 0 to 255 )
        void servo_write(uint8_t id, uint16_t angle, uint16_t time);                    // write to single servo ( id 1 to 6)
        void servo_write6(uint16_t angles[5], uint16_t time, bool floating = false);    // write to 6 servos
        void servo_write6(float32_t angles[5], uint16_t time);                          // write to 6 servos , with downcast from float
        void toggleTorque( bool torque );                                               // turn torque on engines on and off
        void home_position();                                                           // move to the home position

        float32_t servo_read_any(uint8_t id);                                           // read any id from 1 to 6
        float32_t servo_read(uint8_t id);                                               // read any id from 0 to 255
        float32_t* servo_readall();                                                     // read all 6 servos at once

        void servo_set_id(uint8_t id);                                                  // program the servo for id                                                            
            
    private:
        std::array<uint8_t, 13> target;                         // used in cleaning the bus, a buffer of the old destination command
        void bus_cleaner(uint8_t* dest, uint16_t time);         // write onto the bus, only if the coordinates haven't already been sent

        int addr = 0x15;                                            // address of the microcontroller
        int bus = -1;                                             // I2C bus
        int led_bus = -1;
        int led_addr = 0x0d;

        void setRGBColor(uint8_t color);
        void setRGBSpeed(uint8_t speed);
        void setRGBEffect(uint8_t effect);
        void closeRGB();
        void setRGB(uint led, uint8_t r, uint8_t g, uint8_t b);

        

};

