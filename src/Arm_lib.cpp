#include "include/Arm_lib.hpp"

#include <cmath>
#include <iostream>


#ifndef __x86_64
ArmDevice::ArmDevice(){
    this->motorBus_ = open("/dev/i2c-1", O_RDWR);
    if (this->motorBus_ < 0) {
        //throw BusError;
    }

    if (ioctl(this->motorBus_, I2C_SLAVE, this->coprocessorAddress_) < 0) {
        //throw BusError;
    }

    this->ledBus_ = open("/dev/i2c-1", O_RDWR);
    if (this->ledBus_ < 0) {
        //throw BusError;
    }

    if (ioctl(this->ledBus_, I2C_SLAVE, this->hatAddress_) < 0) {
        //throw BusError;
    }

    target_.fill(0);
    }
#else
    #warning Detected to be running under x86, i2c bus calls will be disabled!
    ArmDevice::ArmDevice() = default;
#endif

void ArmDevice::buzz(uint8_t time) const {
    std::array<uint8_t, 2> buf = {0x06U, time};
    if (write(motorBus_, buf.data(), 2) < 0) {
        throw std::runtime_error("i2c bus error for function buzz!");
    }
}

void ArmDevice::noBuzz() const {
    std::array<uint8_t, 2> buf = {0x06U, 0};
    if (write(motorBus_, buf.data(),2) < 0) {
        throw std::runtime_error("i2c bus error for function nobuzz!");
    }
}

[[maybe_unused]] void ArmDevice::servoWrite(uint8_t id, float angle, uint16_t time) {
    if( id == 0 ){
            std::array<float, 6> angles = {angle, angle, angle, angle, angle, angle};
            this->servoWrite6(angles.data(), time);
        }else{
            uint8_t value_h; uint8_t value_l;
            uint8_t time_h; uint8_t time_l;
            uint16_t pos;

            switch (id) {
                case 2:
                case 3:
                case 4:
                    angle = angle;
                    pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900);
                    break;
                case 5:
                    pos = int((3700 - 380) * (angle - 0) / (270 - 0) + 380);
                    break;
                default:
                    pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900);
                    std::cout << pos << '\n';
                    break;
            }

            value_h = (pos >> 8) & 0xFF;
            value_l = pos & 0xFF;

            time_h = (time >> 8) & 0xFF;
            time_l = time & 0xFF;

            std::array<uint8_t, 5> buf = {
                    static_cast<uint8_t>((0x10 + id)),
                    value_h,
                    value_l,
                    time_h,
                    time_l
            };

            write(this->motorBus_, buf.data(), 5);

    }
}

void ArmDevice::servoWrite6(const float *angles, const uint16_t time) {
    std::array<uint8_t, 14> byte_array = {0};
    byte_array[0] = 0x1D;


    // note: angle validity should not be checked here

    for (uint8_t i = 2; i < 13; i += 2) {
        float pos;
        switch (i / 2) {
            case 2:
            case 3:{
                float angle = angles[i / 2 - 1] + 90;
                pos = ((3100.0f - 900.0f) * (float) angle / 180.0f + 900.0f);
                break;
            }
            case 4:{
                float angle = angles[i / 2 - 1] + 180 - 5;
                pos = ((3100.0f - 900.0f) * (float) angle / 180.0f + 900.0f);
                break;
            }
            case 5:
                pos = ((3700.0f - 380.0f) * (float) (angles[i / 2 - 1] + 90 ) / 270.0f + 380.0f);
                break;
            default:
                pos = ((3100.0f - 900.0f) * (angles[i / 2 - 1] + 90 ) / 180.0f + 900.0f);
                break;
        }

        uint16_t p_adj = static_cast<int>(pos);
        byte_array[i - 1] = (p_adj >> 8) & 0xFF;
        byte_array[i] = p_adj & 0xFF;
    }

    busCleaner(byte_array.data(), time);
}

void ArmDevice::toggleTorque(bool torque) const {
    std::array<uint8_t, 2> buf = {0x1A, uint8_t(torque)};

    write(this->motorBus_, buf.data(), 2);    // this is necessary since sometimes
    write(this->motorBus_, buf.data(), 2);    // the command will not register properly

}

void ArmDevice::RGB(uint8_t r, uint8_t g, uint8_t b) const {
    std::array<uint8_t, 4> buf = {0x02, r, g, b};
    write(this->motorBus_, buf.data(), 4);
}

[[maybe_unused]] void ArmDevice::resetMcu() const {
    std::array<uint8_t, 2> buf = {0x05, 0x01};
    write(this->motorBus_, buf.data(), 2);
}

bool ArmDevice::pingServo(uint8_t id) const {
    std::array<uint8_t, 2> buf = {0x38, id};
    write(this->motorBus_, buf.data(), 2);
    uint8_t value = i2c_smbus_read_byte_data(this->motorBus_, 0x38);
    return bool(value);
}

void ArmDevice::buttonMode(int mode) const                              // undocumented function
{
    std::array<uint8_t, 2> buf = {0x03, static_cast<uint8_t> ( mode )};
    write(this->motorBus_, buf.data(), 2);
    write(this->motorBus_, buf.data(), 2);
}

float ArmDevice::servoRead(uint8_t id) const {
    if (((id > 6) || (id < 1))) {
        throw std::runtime_error("Servo ID specified was out of range (1, 6) for ServoRead");
    }

    std::array<uint8_t, 2> buf = {id + 0x30, 0};
    write(this->motorBus_, buf.data(), 2);

    usleep(3000);

    uint16_t pos = i2c_smbus_read_word_data(this->motorBus_, id);
    pos = (pos >> 8 & 0xff) | (pos << 8 & 0xff00);


    float val = NAN;
    switch ((int) id) {
        case 5: {
            val = 270.0f * (static_cast<float> (pos) - 380.0f) / (3700.0f - 380.0f) - 90.0f;
            break;
        }
        case 2:
        case 3:{
            val = std::floor( 180.0f * (static_cast<float> (pos) - 900.0f) / (3100.0f - 900.0f) ) - 90;
            break;
        }
        case 4:{
            val = std::floor( 180.0f * (static_cast<float> (pos) - 900.0f) / (3100.0f - 900.0f) ) - 180 + 4;
            break;
        }
        default: {
            val = 180 * (static_cast<float> (pos) - 900) / (3100 - 900) - 90;
            break;
        }

    }
    return val;
}

std::array<float, 6> ArmDevice::servoReadall() const        // note: this was migrated from returning
{                                                           // a dynamic pointer to a more human array
    std::array<float, 6> values{};
    for (uint8_t i = 1; i <= 6; i++) {

        values[i - 1] = this->servoRead(i);
    }

    return values;
}

void ArmDevice::servoSetId(uint8_t id) const {
    std::array<uint8_t, 2> buf = {0x18, id};
    write(this->motorBus_, buf.data(), 2);
}

void ArmDevice::busCleaner(uint8_t *dest,
                           uint16_t time)           // function introduced by me to prevent spamming the motorBus
{
    std::array<uint8_t, 13> t{};
    std::copy(dest, dest + 13, t.begin());


    if (t != target_) {
        std::array<uint8_t, 3> time_array =
                {
                        0x1E,
                        static_cast<uint8_t>((time >> 8)),
                        static_cast<uint8_t>(time)
                };

        write(this->motorBus_, time_array.data(), 3);
        write(this->motorBus_, dest, 13);
        target_ = t;
    }
}
/*
 *
 * Maybe planned for implementation
void ArmDevice::setRGBColor(uint8_t color) {
}

void ArmDevice::setRGBSpeed(uint8_t speed) {
}

void ArmDevice::setRGBEffect(uint8_t effect) {
}

void ArmDevice::closeRGB() {
}

void ArmDevice::setRGB(uint led, uint8_t r, uint8_t g, uint8_t b) {
}*/
