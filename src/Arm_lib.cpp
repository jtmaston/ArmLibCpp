#include "include/Arm_lib.hpp"

#include <cmath>
#include <iostream>

#define FLOOR_16( x ) ( static_cast<int16_t>(std::floor(x) ))
#define TRUNC_8( x ) (static_cast<int8_t>(x))
#define UNSIGN_8( x ) (static_cast<uint8_t>(x))
#define SIGN_8( x ) (static_cast<int8_t>(x))
#define UNSIGN_16( x ) (static_cast<uint16_t>(x))
#define SIGN_16( x ) (static_cast<int16_t>(x))
#define UPCAST_FLT( x ) (static_cast<float>(x))

#ifndef __x86_64
ArmDevice::ArmDevice(){
    this->motorBus_ = open("/dev/i2c-1", O_RDWR);
    if (this->motorBus_ < 0) {
        //throw BusError;
    }

    if (ioctl(this->motorBus_, UNSIGN_16(I2C_SLAVE), this->coprocessorAddress_) < 0) {
        //throw BusError;
    }

    this->ledBus_ = open("/dev/i2c-1", O_RDWR);
    if (this->ledBus_ < 0) {
        //throw BusError;
    }

    if (ioctl(this->ledBus_, UNSIGN_16(I2C_SLAVE), this->hatAddress_) < 0) {
        //throw BusError;
    }

    target_.fill(0);
    }
#else
    #warning Detected to be running under x86, i2c bus calls will be disabled!
    ArmDevice::ArmDevice() = default;
#endif

void ArmDevice::buzz(int8_t time) const {
    std::array<int8_t, 2U> buf = {0x06, time};
    if (write(motorBus_, buf.data(), 2U) < 0) {
        throw std::runtime_error("i2c bus error for function buzz!");
    }
}

void ArmDevice::noBuzz() const {
    std::array<int8_t, 2U> buf = {0x06, 0};
    if (write(motorBus_, buf.data(),2U) < 0) {
        throw std::runtime_error("i2c bus error for function nobuzz!");
    }
}



inline void checkAndThrow(int8_t expression, int8_t value, const std::string& handle)
{
    if ( expression != value)
    {
        throw std::runtime_error("Communication failure at " + handle);
    }
}


[[maybe_unused]] void ArmDevice::servoWrite(int8_t id, float angle, int16_t time) {
    if( id == 0 ){
            std::array<float, 6U> angles = {angle, angle, angle, angle, angle, angle};
            this->servoWrite6(angles.data(), time);
        }else{
            int8_t value_h; int8_t value_l;
            int8_t time_h; int8_t time_l;
            int16_t pos;

            switch (id) {
                case 2:
                case 3:
                case 4: {
                    angle = angle;
                    pos = FLOOR_16(2200.0F * angle / 180.0F + 900.0F);
                    break;
                }
                case 5: {
                    pos = FLOOR_16(3320.0F * angle / 270.0F + 380.0F);
                    break;
                }
                default: {
                    pos = FLOOR_16(2200.0F * angle / 180.0F + 900.0F);
                    break;
                }
            }

            value_h = TRUNC_8((pos >> 8) & 0xFF);
            value_l = TRUNC_8(pos & 0xFF);

            time_h = TRUNC_8((time >> 8) & 0xFF);
            time_l = TRUNC_8(time & 0xFF);

            std::array<int8_t, 5U> buf = {
                    static_cast<int8_t>((0x10 + id)),
                    value_h,
                    value_l,
                    time_h,
                    time_l
            };

            if ( write(this->motorBus_, buf.data(), 5U) != 5)
            {
                throw std::runtime_error("Communication failure!");
            }

    }
}

void ArmDevice::servoWrite6(const float *angles, const int16_t time) {
    std::array<int8_t, 14U> byte_array = {0};
    byte_array.at(0U) = 0x1D;

    // note: angle validity should not be checked here

    for (int8_t i = 2; i < 13; i += 2) {
        float pos;
        switch (i / 2) {
            case 2:
            case 3:{
                float angle = angles[i / 2 - 1] + 90.0F;
                pos = 2200.0F * angle / 180.0F + 900.0F;
                break;
            }
            case 4:{
                float angle = angles[i / 2 - 1] + 180.0F - 5.0F;
                pos = 2200.0F * angle / 180.0F + 900.0F;
                break;
            }
            case 5:
                pos = 3320.0F *  (angles[i / 2 - 1] + 90.0F ) / 270.0F + 380.0F;
                break;
            default:
                pos = 2200.0F * (angles[i / 2 - 1] + 90.0F ) / 180.0F + 900.0F;
                break;
        }

        int16_t p_adj = FLOOR_16(pos);
        byte_array.at(UNSIGN_8(i - 1)) = TRUNC_8((p_adj >> 8) & 0xFF);
        byte_array.at(UNSIGN_8( i )) = TRUNC_8(p_adj & 0xFF);
    }

    busCleaner(byte_array, time);
}

void ArmDevice::toggleTorque(bool torque) const {
    std::array<int8_t, 2U> buf = {0x1A, SIGN_8(torque)};
    checkAndThrow(
            SIGN_8(write(this->motorBus_, buf.data(), 2U)) +
                        SIGN_8(write(this->motorBus_, buf.data(), 2U)),
            4,
            "toggleTorque"
            );
}



[[maybe_unused]] void ArmDevice::rgb(int8_t r, int8_t g, int8_t b) const {
    std::array<int8_t, 4U> buf = {0x02, r, g, b};
    checkAndThrow(SIGN_8(write(this->motorBus_, buf.data(), 4U)), 4, "RGB");

}

[[maybe_unused]] void ArmDevice::resetMcu() const {
    std::array<int8_t, 2U> buf = {0x05, 0x01};
    checkAndThrow(SIGN_8(write(this->motorBus_, buf.data(), 2U)), 2, "resetMCU");
}

bool ArmDevice::pingServo(int8_t id) const {
    std::array<int8_t, 2U> buf = {0x38, id};
    checkAndThrow(SIGN_8(write(this->motorBus_, buf.data(), 2U)), 2, "pingServo");
    int8_t value = SIGN_8(i2c_smbus_read_byte_data(this->motorBus_, 0x38U));
    return value;
}

void ArmDevice::buttonMode(int mode) const                              // undocumented function
{
    std::array<int8_t, 2U> buf = {0x03, static_cast<int8_t> ( mode )};
    checkAndThrow(SIGN_8(write(this->motorBus_, buf.data(), 2U)), 2, "buttonMode");
    checkAndThrow(SIGN_8(write(this->motorBus_, buf.data(), 2U)), 2, "buttonMode");
    //write(this->motorBus_, buf.data(), 2);
}

float ArmDevice::servoRead(int8_t id) const {
    if (((id > 6) || (id < 1))) {
        throw std::runtime_error("Servo ID specified was out of range (1, 6) for ServoRead");
    }

    std::array<int8_t, 2U> buf = {TRUNC_8(id + 0x30), 0};

    checkAndThrow(SIGN_8(write(this->motorBus_, buf.data(), 2U)), 2, "servoRead");
    (void)usleep(3000U);

    int16_t pos = FLOOR_16(i2c_smbus_read_word_data(this->motorBus_, UNSIGN_8(id)));
    pos = SIGN_16(pos >> 8 & 0xff) | SIGN_16(pos << 8 & 0xff00);

    float val = NAN;
    switch (id) {
        case 5: {
            val = 270.0F * (UPCAST_FLT(pos) - 380.0F) / 3320.0F - 90.0F;
            break;
        }
        case 2:
        case 3:{
            val = 180.0F * (UPCAST_FLT(pos) - 900.0F) / 2200.0F - 90.0F;
            break;
        }
        case 4:{
            val = 180.0F * (UPCAST_FLT(pos) - 900.0F) / 2200.0F - 180.0F + 4.0F;
            break;
        }
        default: {
            val = 180.0F * (UPCAST_FLT(pos) - 900.0F) / 2200.0F - 90.0F;
            break;
        }

    }
    return val;
}

std::array<float, 6U> ArmDevice::servoReadall() const        // note: this was migrated from returning
{                                                           // a dynamic pointer to a more human array
    std::array<float, 6U> values{};
    for (int8_t i = 1; i <= 6; i++) {

        values.at(UNSIGN_8(i - 1)) = this->servoRead(i);
    }
    return values;
}

void ArmDevice::servoSetId(int8_t id) const {
    std::array<int8_t, 2U> buf = {0x18, id};
    checkAndThrow(SIGN_8(write(this->motorBus_, buf.data(), 2U)), 2, "servoSetId");
}

void ArmDevice::busCleaner(const std::array<int8_t, 14U> dest,
                           int16_t time)           // function introduced by me to prevent spamming the motorBus
{
    if (dest != target_) {
        std::array<int8_t, 3U> time_array =
                {
                        0x1E,
                        SIGN_8(time >> 8),
                        SIGN_8(time)
                };
        checkAndThrow(SIGN_8(write(this->motorBus_, time_array.data(), 3U)), 3, "busCleaner");
        checkAndThrow(SIGN_8(write(this->motorBus_, dest.data(), 13U)), 13, "busCleaner");
        target_ = dest;
    }
}

void ArmDevice::servoWriteAny(int8_t id, int16_t angle, int16_t time) const {

}
/*
 *
 * Maybe planned for implementation
void ArmDevice::setRGBColor(int8_t color) {
}

void ArmDevice::setRGBSpeed(int8_t speed) {
}

void ArmDevice::setRGBEffect(int8_t effect) {
}

void ArmDevice::closeRGB() {
}

void ArmDevice::setRGB(uint led, int8_t r, int8_t g, int8_t b) {
}*/
