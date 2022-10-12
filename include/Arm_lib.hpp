#ifndef ROBOTDASHBOARD_ARM_LIB_HPP
#define ROBOTDASHBOARD_ARM_LIB_HPP

#include <cstdlib>
#include <cstdio>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include <cmath>
#include <array>
#include <vector>

extern "C"
{
#include "linux/i2c-dev.h"
#include <i2c/smbus.h>
}

#define RAD 0.0174533

class ArmDevice {
public:
    std::vector<std::vector<float>> learnedAngles_;

    ArmDevice();


    [[maybe_unused]] void buzz(uint8_t time = 10) const;                  // start the buzzer
    [[maybe_unused]] void noBuzz() const;                                 // stop the buzzer

    [[maybe_unused]] void rgb(uint8_t r, uint8_t g, uint8_t b) const;     // set the RGB LED value
    [[maybe_unused]] void resetMcu() const;                               // reset microcontroller
    [[maybe_unused]] [[maybe_unused]] [[nodiscard]] bool pingServo(int8_t id) const;                        // ping to test if servo is available
    [[maybe_unused]] void buttonMode();                                   // undocumented function

    [[maybe_unused]] void servoWriteAny(uint8_t id, uint16_t angle,
                                        uint16_t time) const;              // write value to any servo ( id 0 to 255 )
    [[maybe_unused]] void servoWrite(uint8_t id, uint16_t angle, uint16_t time);            // write to single servo ( id 1 to 6)
    [[maybe_unused]] void servoWrite(uint8_t id, float angle, uint16_t time);               // write to single servo ( id 1 to 6)
    [[maybe_unused]] void servoWrite6(const uint16_t angles[6], uint16_t time);             // write to 6 servos
    void servoWrite6(const float angles[6], uint16_t time);                // write to 6 servos , with cast to int
    void toggleTorque(bool torque) const;                                  // turn torque on engines on and off


    [[maybe_unused]] [[nodiscard]] float servoReadAny(uint8_t id) const;                    // read any id from 1 to 6
    [[nodiscard]] float servoRead(uint8_t id) const;                       // read any id from 0 to 255
    [[nodiscard]] float* servoReadall() const;                             // read all 6 servos at once

    [[maybe_unused]] void servoSetId(uint8_t id) const;                                     // program the servo for id
    std::array<uint8_t, 13> target_{};

    void busCleaner(uint8_t *dest, uint16_t time);

    int addr_ = 0x15;                                                       // address of the microcontroller
    int bus_ = -1;                                                          // I2C bus_
    int ledBus_ = -1;
    int ledAddr_ = 0x0d;

    // void setRGBColor(uint8_t color);                     TODO: implement these
    // void setRGBSpeed(uint8_t speed);
    // void setRGBEffect(uint8_t effect);
    // void closeRGB();
    // void setRGB(uint led, uint8_t r, uint8_t g, uint8_t b);
};

#endif