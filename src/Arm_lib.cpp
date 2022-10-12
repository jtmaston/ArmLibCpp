#include "include/Arm_lib.hpp"
#include <iostream>

ArmDevice::ArmDevice() {
    this->bus_ = open("/dev/i2c-1", O_RDWR);
    this->addr_ = 0x15;
    if (this->bus_ < 0) {
        // throw std::runtime_error("Bus error!");
    }

    if (ioctl(this->bus_, I2C_SLAVE, this->addr_) < 0) {
        // throw std::runtime_error("Bus error!");
    }

    this->ledBus_ = open("/dev/i2c-1", O_RDWR);
    this->ledAddr_ = 0x0d;
    if (this->ledBus_ < 0) {
        // throw std::runtime_error("Bus error!");
    }

    if (ioctl(this->ledBus_, I2C_SLAVE, this->ledAddr_) < 0) {
        // throw std::runtime_error("Bus error!");
    }

    target_.fill(0);
}

[[maybe_unused]] void ArmDevice::buzz(uint8_t time) const {
    uint8_t buf[] = {0x06, time};
    write(bus_, buf, 2);
}

[[maybe_unused]] void ArmDevice::noBuzz() const {
    uint8_t buf[] = {0x06, 0};
    write(bus_, buf, 2);
}

[[maybe_unused]] void ArmDevice::servoWrite(uint8_t id, float angle, uint16_t time) {
    if (id == 0) {
        float angles[] = {angle, angle, angle, angle, angle, angle};
        this->servoWrite6(angles, time);
    } else {
        uint8_t value_h, value_l, time_h, time_l;
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
                break;
        }

        value_h = (pos >> 8) & 0xFF;
        value_l = pos & 0xFF;

        time_h = (time >> 8) & 0xFF;
        time_l = time & 0xFF;

        uint8_t buf[] = {
                static_cast<uint8_t>((0x10 + id)),
                value_h,
                value_l,
                time_h,
                time_l
        };

        write(this->bus_, buf, 5);
    }
}


[[maybe_unused]] void ArmDevice::servoWrite(uint8_t id, uint16_t angle, uint16_t time) {
    if (id == 0) {
        float angles[] = {angle, angle, angle, angle, angle, angle};
        this->servoWrite6(angles, time);
    } else {
        uint8_t value_h, value_l, time_h, time_l;
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

        uint8_t buf[] = {
                static_cast<uint8_t>((0x10 + id)),
                value_h,
                value_l,
                time_h,
                time_l
        };

        write(this->bus_, buf, 5);
    }
}


void ArmDevice::servoWrite6(const float angles[6], uint16_t time) {
    uint8_t byte_array[14] = {0};
    byte_array[0] = 0x1D;


    bool flag = false;
    for (uint8_t i = 0; i < 6; i++) {
        flag |= ((angles[i] > 180) || (angles[i] < 0));
    }

    /*if (flag)                 TODO: this needs to be reimplemented with limits
    {
        AngleError_C err;
        throw err;
    }*/

    for (uint8_t i = 2; i < 13; i += 2) {
        float pos;
        switch (i / 2) {
            case 2:
            case 3:
            case 4: {

                float angle = angles[i / 2 - 1];
                pos = ((3100.0f - 900.0f) * (float) angle / 180.0f + 900.0f);
                break;
            }
            case 5:
                pos = ((3700.0f - 380.0f) * (float) angles[i / 2 - 1] / 270.0f + 380.0f);
                break;
            default:
                pos = ((3100.0f - 900.0f) * angles[i / 2 - 1] / 180.0f + 900.0f);
                break;
        }

        uint16_t p_adj = static_cast<int>(pos);
        byte_array[i - 1] = (p_adj >> 8) & 0xFF;
        byte_array[i] = p_adj & 0xFF;
    }

    busCleaner(byte_array, time);
}

void ArmDevice::toggleTorque(bool torque) const {
    uint8_t buf[2] = {0x1A, uint8_t(torque)};
    write(this->bus_, buf, 2);
}

[[maybe_unused]] void ArmDevice::rgb(uint8_t r, uint8_t g, uint8_t b) const {
    uint8_t buf[4] = {0x02, r, g, b};
    write(this->bus_, buf, 4);
}

[[maybe_unused]] void ArmDevice::resetMcu() const {
    uint8_t buf[2] = {0x05, 0x01};
    write(this->bus_, buf, 2);
}

[[maybe_unused]] bool ArmDevice::pingServo(int8_t id) const {
    uint8_t buf[4] = {0x38, id};
    write(this->bus_, buf, 2);
    uint8_t value = i2c_smbus_read_byte_data(this->bus_, 0x38);
    return bool(value);
}

[[maybe_unused]] void ArmDevice::buttonMode()                              // undocumented function
{
    //uint8_t buf[4] { 0x03, static_cast<uint8_t> ( mode ) };
    // TODO: me
}

float ArmDevice::servoRead(uint8_t id) const {
    if (((id > 6) || (id < 1)) == 1) {
        throw std::runtime_error("ID not mapped!");
    }

    id += 0x30;
    uint8_t buf[3] = {id, 0};
    write(this->bus_, buf, 2);

    float val;

    usleep(3000);

    uint32_t read_from_bus = i2c_smbus_read_word_data(this->bus_, id);
    auto pos = (float) ((read_from_bus >> 8 & 0xff) | (read_from_bus << 8 & 0xff00));

    id -= 0x30;

    switch ((int) id) {
        case 5: {
            val = 270.0f * (pos - 380.0f) / (3700 - 380);
            break;
        }
        case 2:
        case 3:
        case 4: {
            //std::cout << "Adjusted";
            val = 180.0f * (pos - 900) / (3100 - 900);
            val = 180 - val;
            break;
        }
        default: {
            //std::cout << "Def";
            val = 180.0f * (pos - 900) / (3100 - 900);
            break;
        }

    }
    return val;
}

float *ArmDevice::servoReadall() const {
    auto *values = new float[6];
    for (uint8_t i = 1; i <= 6; i++) {

        values[i - 1] = this->servoRead(i);
        //std::cout << values[i - 1] << " ";
    }
    //std::cout << '\n';


    return values;
}

[[maybe_unused]] float ArmDevice::servoReadAny(uint8_t id) const {
    if (id < 1 or id > 250) {
        throw std::runtime_error("Servo ID is unmapped!");
    }

    uint8_t buf[3] = {0x37, id};
    write(this->bus_, buf, 2);
    usleep(3000);
    uint32_t read_from_bus = i2c_smbus_read_word_data(this->bus_, id);
    auto pos = float((read_from_bus >> 8 & 0xff) | (read_from_bus << 8 & 0xff00));

    return (float) (180.0 * (pos - 900.0) / (3100.0 - 900.0));
}

[[maybe_unused]] void ArmDevice::servoWriteAny(uint8_t id, uint16_t angle, uint16_t time) const {
    if (id == 0)
        return;

    auto pos = uint16_t((3100 - 900) * angle / 180 + 900);
    uint8_t buf[6] =
            {
                    static_cast<uint8_t> (0x19),
                    id,
                    static_cast<uint8_t>((pos >> 8)),
                    static_cast<uint8_t> (pos),
                    static_cast<uint8_t> ((time >> 8)),
                    static_cast<uint8_t> (time)
            };
    write(this->bus_, buf, 6);
}

[[maybe_unused]] void ArmDevice::servoSetId(uint8_t id) const {
    uint8_t buf[2] = {0x18, id};
    write(this->bus_, buf, 2);
}

void
ArmDevice::busCleaner(uint8_t *dest, uint16_t time)           // function introduced by me to prevent spamming the bus_
{
    std::array<uint8_t, 13> t = {0};
    std::copy(dest, dest + 13, t.begin());


    if (t != target_) {
        uint8_t time_array[3] =
                {
                        0x1E,
                        static_cast<uint8_t>((time >> 8)),
                        static_cast<uint8_t>(time)
                };

        write(this->bus_, time_array, 3);
        write(this->bus_, dest, 13);
        target_ = t;
    }
}

[[maybe_unused]] [[maybe_unused]] void ArmDevice::servoWrite6(const uint16_t *angles, uint16_t time) {
    uint8_t byte_array[14] = {0};
    byte_array[0] = 0x1D;


    bool flag = false;
    for (uint8_t i = 0; i < 6; i++) {
        flag |= ((angles[i] > 180) || (angles[i] < 0));
    }

    /*if (flag)                 TODO: this needs to be reimplemented with limits
    {
        AngleError_C err;
        throw err;
    }*/

    for (uint8_t i = 2; i < 13; i += 2) {
        float pos;
        switch (i / 2) {
            case 2:
            case 3:
            case 4: {

                float angle = angles[i / 2 - 1];
                pos = ((3100.0f - 900.0f) * (float) angle / 180.0f + 900.0f);
                break;
            }
            case 5:
                pos = ((3700.0f - 380.0f) * (float) angles[i / 2 - 1] / 270.0f + 380.0f);
                break;
            default:
                pos = ((3100.0f - 900.0f) * (float) angles[i / 2 - 1] / 180.0f + 900.0f);
                break;
        }

        uint16_t p_adj = static_cast<int>(pos);
        byte_array[i - 1] = (p_adj >> 8) & 0xFF;
        byte_array[i] = p_adj & 0xFF;
    }

    busCleaner(byte_array, time);
}




