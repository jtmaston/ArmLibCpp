#pragma once

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

class ArmDevice
{
    public:
    ArmDevice();


    void buzz(int8_t time = 10) const ;                                                   // start the buzzer
    void noBuzz() const;                                                                  // stop the buzzer

    [[maybe_unused]] void rgb(int8_t r, int8_t g, int8_t b) const;                                      // set the rgb led value
    void resetMcu() const;                                                               // reset microcontroller
    [[nodiscard]] bool pingServo(
            int8_t id) const;                                                    // ping to test if servo is available
    void buttonMode(int mode) const;                                                    // undocumented function

    void servoWriteAny(int8_t id, int16_t angle,
                       int16_t time) const;                // write value to any servo ( id 0 to 255 )         // write to single servo ( id 1 to 6)
     void servoWrite(int8_t id, float angle,
                    int16_t time);                    // write to single servo ( id 1 to 6)         // write to 6 servos
    void servoWrite6(std::vector<float> angles,
                     int16_t time);                          // write to 6 servos , with downcast from float
    void
    toggleTorque(bool torque) const;                                               // turn torque on engines on and off


    //[[nodiscard]] float
    //servoReadAny(int8_t id) const;                                           // read any id from 1 to 6
    [[nodiscard]] float
    servoRead(int8_t id) const;                                               // read any id from 0 to 255
    [[nodiscard]] std::array<float, 6U>
    servoReadall() const;                                                     // read all 6 servos at once

    void servoSetId (int8_t id) const;                                                  // initializeInterpreterThread the servo for id
                            // used in cleaning the motorBus, a buffer of the old destination command
    void busCleaner(std::array<int8_t, 14U> dest,
                    int16_t time);         // write onto the motorBus, only if the coordinates haven't already been sent
private:
    std::array<int8_t, 14U> target_{};
    const int coprocessorAddress_ = 0x15;
    const int hatAddress_ = 0x0d;
    int motorBus_ = -1;                                             // I2C motorBus
    int ledBus_ = -1;


    /*void setRGBColor(int8_t color);
    void setRGBSpeed(int8_t speed);
    void setRGBEffect(int8_t effect);
    void closeRGB();
    void setRGB(int led, int8_t r, int8_t g, int8_t b);*/
};

