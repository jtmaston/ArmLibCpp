#include "include/Arm_lib.hpp"

#include <cmath>
#include <iostream>

#define FLOOR_16(x) ( static_cast<int16_t>(std::floor(x) ))
#define TRUNC_8(x) (static_cast<int8_t>(x))
#define UNSIGN_8(x) (static_cast<uint8_t>(x))
#define SIGN_8(x) (static_cast<int8_t>(x))
#define UNSIGN_16(x) (static_cast<uint16_t>(x))
#define SIGN_16(x) (static_cast<int16_t>(x))
#define UPCAST_FLT(x) (static_cast<float>(x))

#include <filesystem>

namespace fs = std::filesystem;

#ifndef __x86_64

ArmDevice::ArmDevice() {

    std::vector<std::string> port_names;
    fs::path p("/dev/serial/by-id");
    if (!exists(p)) {
        throw std::runtime_error("No serial port!");
    } else {
        for (auto de: fs::directory_iterator(p)) {
            if (is_symlink(de.symlink_status())) {
                fs::path symlink_points_at = read_symlink(de);
                fs::path canonical_path = fs::canonical(p / symlink_points_at);

                motorBus_ = open(canonical_path.generic_string().c_str(), O_RDWR);
            }
        }
    }

    if (tcgetattr(motorBus_, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
    //tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
    tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    if (tcsetattr(motorBus_, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    /*uint8_t buf[] = "VR\r\n";
    write(motorBus_, buf, sizeof(buf));

    char read_buf[256];
    int n = read(motorBus_, &read_buf, sizeof(read_buf));
    if(strncmp(read_buf, "RV-E", 4) != 0)
        std::cout << "Unsupported robot model!";*/

    usleep(1e3);
    uint8_t goHome[] = "MP 574.84,0.00,565.45,-180.00,69.99,-180.00,R,A,N\r\n";
    write(motorBus_, goHome, sizeof(goHome));

}

inline size_t writeToBus(int fd, const void *buf, size_t n) {
    return 0;
    //return write(fd, buf, n);
}

inline int readWordFromBus(int fd, uint8_t command) {
    return 0;
    //return i2c_smbus_read_word_data(fd, command);
}

inline int readByteFromBus(int fd, uint8_t command) {
    return 0;
    //return i2c_smbus_read_byte_data(fd, command);
}

#else
#warning Detected to be running under x86, i2c bus calls will be disabled!
ArmDevice::ArmDevice() = default;
inline size_t writeToBus(int fd, const void* buf, size_t n)
{
    return n;
}

inline int readWordFromBus(int fd, uint8_t command)
{
    return 0;
}
inline int readByteFromBus(int fd, uint8_t command)
{
    return 0;
}

#endif


void ArmDevice::buzz(int8_t time) const {
    std::array<int8_t, 2U> buf = {0x06, time};
    if (writeToBus(motorBus_, buf.data(), 2U) < 0) {
        throw std::runtime_error("i2c bus error for function buzz!");
    }
}

void ArmDevice::noBuzz() const {
    std::array<int8_t, 2U> buf = {0x06, 0};
    if (writeToBus(motorBus_, buf.data(), 2U) < 0) {
        throw std::runtime_error("i2c bus error for function nobuzz!");
    }
}


inline void checkAndThrow(int8_t expression, int8_t value, const std::string &handle) {
    /*if (expression != value) {
        throw std::runtime_error("Communication failure at " + handle);
    }*/
}


[[maybe_unused]] void ArmDevice::servoWrite(int8_t id, float angle, int16_t time) {
    if (id == 0) {
        std::vector<float> angles = {angle, angle, angle, angle, angle, angle};
        this->servoWrite6(angles, time);
    } else {
        int8_t value_h;
        int8_t value_l;
        int8_t time_h;
        int8_t time_l;
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

        if (writeToBus(this->motorBus_, buf.data(), 5U) != 5) {
            throw std::runtime_error("Communication failure!");
        }

    }
}

void ArmDevice::servoWrite6(std::vector<float> angles, const int16_t time) {
    std::array<int8_t, 14U> byte_array = {0};
    byte_array.at(0U) = 0x1D;

    // note: angle validity should not be checked here

    for (int8_t i = 2; i < 13; i += 2) {
        float pos;
        switch (i / 2) {
            case 2:
            case 3: {
                float angle = angles[i / 2 - 1] + 90.0F;
                pos = 2200.0F * angle / 180.0F + 900.0F;
                break;
            }
            case 4: {
                float angle = angles[i / 2 - 1] + 180.0F - 5.0F;
                pos = 2200.0F * angle / 180.0F + 900.0F;
                break;
            }
            case 5:
                pos = 3320.0F * (angles[i / 2 - 1] + 90.0F) / 270.0F + 380.0F;
                break;
            default:
                pos = 2200.0F * (angles[i / 2 - 1] + 90.0F) / 180.0F + 900.0F;
                break;
        }

        int16_t p_adj = FLOOR_16(pos);
        byte_array.at(UNSIGN_8(i - 1)) = TRUNC_8((p_adj >> 8) & 0xFF);
        byte_array.at(UNSIGN_8(i)) = TRUNC_8(p_adj & 0xFF);
    }

    busCleaner(byte_array, time);
}

void ArmDevice::toggleTorque(bool torque) const {
    std::array<int8_t, 2U> buf = {0x1A, SIGN_8(torque)};
    checkAndThrow(
            SIGN_8(writeToBus(this->motorBus_, buf.data(), 2U)) +
            SIGN_8(writeToBus(this->motorBus_, buf.data(), 2U)),
            4,
            "toggleTorque"
    );
}


[[maybe_unused]] void ArmDevice::rgb(int8_t r, int8_t g, int8_t b) const {
    std::array<int8_t, 4U> buf = {0x02, r, g, b};
    checkAndThrow(SIGN_8(writeToBus(this->motorBus_, buf.data(), 4U)), 4, "RGB");

}

[[maybe_unused]] void ArmDevice::resetMcu() const {
    std::array<int8_t, 2U> buf = {0x05, 0x01};
    checkAndThrow(SIGN_8(writeToBus(this->motorBus_, buf.data(), 2U)), 2, "resetMCU");
}

bool ArmDevice::pingServo(int8_t id) const {
    std::array<int8_t, 2U> buf = {0x38, id};
    checkAndThrow(SIGN_8(writeToBus(this->motorBus_, buf.data(), 2U)), 2, "pingServo");
    int8_t value = SIGN_8(readByteFromBus(this->motorBus_, 0x38U));
    return value;
}

void ArmDevice::buttonMode(int mode) const                              // undocumented function
{
    std::array<int8_t, 2U> buf = {0x03, static_cast<int8_t> ( mode )};
    checkAndThrow(SIGN_8(writeToBus(this->motorBus_, buf.data(), 2U)), 2, "buttonMode");
    checkAndThrow(SIGN_8(writeToBus(this->motorBus_, buf.data(), 2U)), 2, "buttonMode");
    //write(this->motorBus_, buf.data(), 2);
}

float ArmDevice::servoRead(int8_t id) const {
    if (((id > 6) || (id < 1))) {
        throw std::runtime_error("Servo ID specified was out of range (1, 6) for ServoRead");
    }

    std::array<int8_t, 2U> buf = {TRUNC_8(id + 0x30), 0};

    checkAndThrow(SIGN_8(writeToBus(this->motorBus_, buf.data(), 2U)), 2, "servoRead");
    (void) usleep(3000U);

    int16_t pos = FLOOR_16(readWordFromBus(this->motorBus_, UNSIGN_8(id)));
    pos = SIGN_16(pos >> 8 & 0xff) | SIGN_16(pos << 8 & 0xff00);

    float val = NAN;
    switch (id) {
        case 5: {
            val = 270.0F * (UPCAST_FLT(pos) - 380.0F) / 3320.0F - 90.0F;
            break;
        }
        case 2:
        case 3: {
            val = 180.0F * (UPCAST_FLT(pos) - 900.0F) / 2200.0F - 90.0F;
            break;
        }
        case 4: {
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
    checkAndThrow(SIGN_8(writeToBus(this->motorBus_, buf.data(), 2U)), 2, "servoSetId");
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
        checkAndThrow(SIGN_8(writeToBus(this->motorBus_, time_array.data(), 3U)), 3, "busCleaner");
        checkAndThrow(SIGN_8(writeToBus(this->motorBus_, dest.data(), 13U)), 13, "busCleaner");
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
