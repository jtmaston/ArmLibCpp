#include "include/Arm_lib.hpp"
#include <iostream>

ArmDevice::ArmDevice()
{
    BusError_C BusError;
    this -> bus = open("/dev/i2c-1", O_RDWR);
    this -> addr = 0x15;
    if (this->bus < 0)
    {
        //throw BusError;
    }

    if (ioctl(this -> bus, I2C_SLAVE, this -> addr) < 0)
    {
        //throw BusError;
    }

    this -> led_bus = open("/dev/i2c-1", O_RDWR);
    this -> led_addr = 0x0d;
    if (this->led_bus < 0)
    {
        //throw BusError;
    }

    if (ioctl(this -> led_bus, I2C_SLAVE, this -> led_addr) < 0)
    {
        //throw BusError;
    }

    target.fill(0);
}

void ArmDevice::buzz(uint8_t time)
{
    uint8_t buf[] = { 0x06, time };
    write(bus, buf, 2);
}

void ArmDevice::noBuzz()
{
    uint8_t buf[] = {0x06, 0};
    write(bus, buf, 2);
}

void ArmDevice::servo_write(uint8_t id, uint16_t angle, uint16_t time)
{
    switch( id )
    {
    case 0:
    {
        uint16_t angles[] = { angle, angle, angle, angle, angle, angle };
        this -> servo_write6(angles, time);
        break;
    }

    default:
        {
            uint8_t value_h, value_l, time_h, time_l;
            uint16_t pos;

            //std::cout << pos;

            switch(id)
                {
                case 2:
                case 3:
                case 4:
                    angle = 180 - angle;
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
                static_cast<uint8_t>( (0x10 + id) ) ,
                value_h,
                value_l,
                time_h,
                time_l
            };

            write(this -> bus, buf, 5);
            break;
        }
    }
}

void ArmDevice::servo_write6(float angles[6], uint16_t time)
{
    uint16_t* angle_t = new uint16_t[6];

    for(int i = 0; i < 6; i++)
        angle_t[i] = angles[i];

    servo_write6(angle_t, time, true);
}

void ArmDevice::servo_write6(uint16_t angles[6], uint16_t time, bool floating)
{
    uint8_t bytearr[14] = {0};
    bytearr[0] = 0x1D;
    

    bool flag = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        flag |= ((angles[i] > 180) || (angles[i] < 0));
    }

    /*if (flag)                 TODO: this needs to be reimplemented with limits
    {
        AngleError_C err;
        throw err;
    }*/

    for (uint8_t i = 2; i < 13; i += 2)
    {
        float pos = 0;
        switch (i / 2)
        {
        case 2:
        case 3:
        case 4:
        {
            uint16_t angle = 180 - angles[i / 2 - 1];
            pos = ((3100.0f - 900.0f) * (float)angle / 180.0f + 900.0f);
            break;
        }
        case 5:
            pos = (3700.0f - 380.0f) * (float)angles[i / 2 - 1] / 270.0f  + 380.0f;
            break;
        default:
            pos = (3100.0f - 900.0f) * angles[i / 2 - 1]/ 180.0f + 900.0f;
            break;
        }
        uint16_t p_adj = trunc(pos);
        bytearr[i - 1] = (p_adj >> 8) & 0xFF;
        bytearr[i] = p_adj & 0xFF;
    }

    bus_cleaner(bytearr, time);
}

void ArmDevice::toggleTorque(bool torque)
{
    uint8_t buf[2] = { 0x1A, uint8_t(torque) };
    write( this -> bus, buf, 2);
}
void ArmDevice::rgb(uint8_t r, uint8_t g, uint8_t b)
{   
    uint8_t buf[4] = { 0x02, r, g ,b};
    write( this -> bus, buf, 4);
}

void ArmDevice::reset_mcu()
{
    uint8_t buf[2] = { 0x05, 0x01 };
    write( this -> bus, buf, 2);
}

bool ArmDevice::ping_servo(uint8_t id)
{
    uint8_t buf[4] = { 0x38, id };
    write( this -> bus, buf, 2);
    uint8_t value = i2c_smbus_read_byte_data(this -> bus, 0x38);
    return bool ( value );
}
void ArmDevice::button_mode(bool mode)                              // undocumented function
{
    //uint8_t buf[4] { 0x03, static_cast<uint8_t> ( mode ) };
    // TODO: me
}

float ArmDevice::servo_read(uint8_t id)
{
    if (((id > 6) || (id < 1)) == 1)
    {
        UnmappedError err;
        throw err;
    }
    
    id += 0x30;
    uint8_t buf[3] = {id, 0};
    uint16_t pos = write(this->bus, buf, 2);

    float val;

    usleep(3000);

    pos = i2c_smbus_read_word_data(this->bus, id);
    pos = (pos >> 8 & 0xff) | (pos << 8 & 0xff00);

    id -= 0x30;

    switch ( (int)id )
    {
        case 5:
            {
                val = (270 - 0) * (pos - 380) / (3700 - 380);
                break;
            }
        case 2:
        case 3:
        case 4:
            {
                //std::cout << "Adjusted";
                val = 180 * (pos - 900) / (3100 - 900);
                val = 180 - val;
                break;
            }
        default:
            {
                //std::cout << "Def";
                val = 180 * (pos - 900) / (3100 - 900);
                break;
            }
        
    }
    return val;
}

float *ArmDevice::servo_readall()
{
    float* values = new float[6];
    for (uint8_t i = 1; i <= 6; i++){

        values[i - 1] = this->servo_read(i);
        //std::cout << values[i - 1] << " ";
    }
    //std::cout << '\n';
    

    return values;
}

float ArmDevice::servo_read_any(uint8_t id)
{
    if (id < 1 or id > 250)
    {
        UnmappedError err;
        throw err;
    }

    uint8_t buf[3] = {0x37, id};
    uint16_t pos = write(this->bus, buf, 2);
    usleep(3000);
    pos = i2c_smbus_read_word_data(this->bus, id);
    pos = (pos >> 8 & 0xff) | (pos << 8 & 0xff00);

    return 180.0 * (pos - 900.0) / (3100.0 - 900.0);
}

void ArmDevice::servo_write_any(uint8_t id, uint16_t angle, uint16_t time)
{
    if ( id == 0 )
        return;
    
    uint16_t pos = uint16_t((3100 - 900) * angle / 180 + 900);
    uint8_t buf[6]  = 
    {
    static_cast<uint8_t> (0x19),
    id,
    static_cast<uint8_t>( (pos >> 8) ),
    static_cast<uint8_t> (pos), 
    static_cast<uint8_t> (( time >> 8 )), 
    static_cast<uint8_t> (time)
    };
    write(this -> bus, buf, 6);
}

void ArmDevice::servo_set_id(uint8_t id)
{
    uint8_t buf[2] = { 0x18, id};
    write(this -> bus, buf, 2);
}

void ArmDevice::bus_cleaner(uint8_t* dest, uint16_t time)           // function introduced by me to prevent spamming the bus
{
    std::array<uint8_t, 13> t;
    std::copy(dest, dest+13, t.begin());


    if( t != target)
    {
        uint8_t timearr[3] = 
        { 
            0x1E, 
            static_cast<uint8_t>((time >> 8)),
            static_cast<uint8_t>(time)
         };

        write(this -> bus, timearr, 3);
        write(this -> bus, dest, 13);
        target = std::move(t);
    }
}