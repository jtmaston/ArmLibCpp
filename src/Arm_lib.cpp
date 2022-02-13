#include "Arm_lib.hpp"
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
    send(buf, 2);
}

void ArmDevice::noBuzz()
{
    uint8_t buf[] = {0x06, 0};
    send(buf, 2);
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
                    std::cout << "Branch 1";
                    break;
                case 5:
                    pos = int((3700 - 380) * (angle - 0) / (270 - 0) + 380);
                    std::cout << "Branch 2";
                    break;
                default:
                    std::cout <<"Default";
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

void ArmDevice::servo_write6(float32_t angles[6], uint16_t time)
{
    uint16_t* angle_t = new uint16_t[6];

    for(int i = 0; i < 6; i++)
        angle_t[i] = angles[i];

    servo_write6(angle_t, time, true);
}

void ArmDevice::servo_write6(uint16_t angles[6], uint16_t time, bool floating)
{
    //toggleTorque(true);
    uint8_t bytearr[14] = {0};
    bytearr[0] = 0x1D;
    

    bool flag = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        flag |= ((angles[i] > 180) || (angles[i] < 0));
        //std::cout << angles[i] << " ";
    }

    /*if (flag)                 TODO: this needs to be reimplemented with limits
    {
        AngleError_C err;
        throw err;
    }*/

    for (uint8_t i = 2; i < 13; i += 2)
    {
        //std::cout << (int)angles[i / 2 - 1] << " ";
        float pos = 0;
        switch (i / 2)
        {
        case 2:
        case 3:
        case 4:
        {
            //std::cout << "br1" << '\n';
            uint16_t angle = 180 - angles[i / 2 - 1];
            pos = ((3100.0f - 900.0f) * (float)angle / 180.0f + 900.0f);
            break;
        }
        case 5:
            //std::cout << "br2"<< '\n';
            pos = (3700.0f - 380.0f) * (float)angles[i / 2 - 1] / 270.0f  + 380.0f;
            break;
        default:
            //std::cout << "br3"<< '\n';
            pos = (3100.0f - 900.0f) * angles[i / 2 - 1]/ 180.0f + 900.0f;
            break;
        }
        uint16_t p_adj = trunc(pos);
        bytearr[i - 1] = (p_adj >> 8) & 0xFF;
        bytearr[i] = p_adj & 0xFF;
    }
    //std::cout << '\n';

    

    //for (int i = 1; i < 13; i++)
        //std::cout << (int)bytearr[i] << " ";

    //std::cout << '\n';

    //std::cout << (int)timearr[1] << " " << (int)timearr[2] << '\n';

    //int t = write(this->bus, timearr, 3);
    bus_cleaner(bytearr, time);
    //t = write(this->bus, bytearr, 13);
    //if (floating)
        //delete angles;
}

bool ArmDevice::send( uint8_t buffer[100], uint16_t buflen)
{
    if (write(this -> bus, buffer, buflen) != 2)
    {
        return 0;
    }
    return 1;
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

void ArmDevice::neon_multiply(float32_t *T1, float32_t *T2, float32_t *T3) {

        // these are the columns A
        float32x4_t T1_0;
        float32x4_t T1_1;
        float32x4_t T1_2;
        float32x4_t T1_3;
        
        // these are the columns B
        float32x4_t T2_0;
        float32x4_t T2_1;
        float32x4_t T2_2;
        float32x4_t T2_3;
        
        // these are the columns C
        float32x4_t T3_0;
        float32x4_t T3_1;
        float32x4_t T3_2;
        float32x4_t T3_3;
        
        T1_0 = vld1q_f32(T1);
        T1_1 = vld1q_f32(T1+4);
        T1_2 = vld1q_f32(T1+8);
        T1_3 = vld1q_f32(T1+12);
        
        // Zero accumulators for C values
        T3_0 = vmovq_n_f32(0);
        T3_1 = vmovq_n_f32(0);
        T3_2 = vmovq_n_f32(0);
        T3_3 = vmovq_n_f32(0);
        
        // Multiply accumulate in 4x1 blocks, i.e. each column in C
        T2_0 = vld1q_f32(T2);
        T3_0 = vfmaq_laneq_f32(T3_0, T1_0, T2_0, 0);
        T3_0 = vfmaq_laneq_f32(T3_0, T1_1, T2_0, 1);
        T3_0 = vfmaq_laneq_f32(T3_0, T1_2, T2_0, 2);
        T3_0 = vfmaq_laneq_f32(T3_0, T1_3, T2_0, 3);
        vst1q_f32(T3, T3_0);
        
        T2_1 = vld1q_f32(T2+4);
        T3_1 = vfmaq_laneq_f32(T3_1, T1_0, T2_1, 0);
        T3_1 = vfmaq_laneq_f32(T3_1, T1_1, T2_1, 1);
        T3_1 = vfmaq_laneq_f32(T3_1, T1_2, T2_1, 2);
        T3_1 = vfmaq_laneq_f32(T3_1, T1_3, T2_1, 3);
        vst1q_f32(T3+4, T3_1);
        
        T2_2 = vld1q_f32(T2+8);
        T3_2 = vfmaq_laneq_f32(T3_2, T1_0, T2_2, 0);
        T3_2 = vfmaq_laneq_f32(T3_2, T1_1, T2_2, 1);
        T3_2 = vfmaq_laneq_f32(T3_2, T1_2, T2_2, 2);
        T3_2 = vfmaq_laneq_f32(T3_2, T1_3, T2_2, 3);
        vst1q_f32(T3+8, T3_2);
        
        T2_3 = vld1q_f32(T2+12);
        T3_3 = vfmaq_laneq_f32(T3_3, T1_0, T2_3, 0);
        T3_3 = vfmaq_laneq_f32(T3_3, T1_1, T2_3, 1);
        T3_3 = vfmaq_laneq_f32(T3_3, T1_2, T2_3, 2);
        T3_3 = vfmaq_laneq_f32(T3_3, T1_3, T2_3, 3);
        vst1q_f32(T3+12, T3_3);
}

void ArmDevice::c_multiply(float32_t *A, float32_t *B, float32_t *C)
{
    int n = 4, m = 4, k = 4;
    for (int i_idx = 0; i_idx < n; i_idx++)
    {
        for (int j_idx = 0; j_idx < m; j_idx++)
        {
            C[n * j_idx + i_idx] = 0;
            for (int k_idx = 0; k_idx < k; k_idx++)
            {
                C[n * j_idx + i_idx] += A[n * k_idx + i_idx] * B[k * j_idx + k_idx];
            }
        }
    }
}


void ArmDevice::rotateX(uint8_t num, float32_t* target)
{
    float32_t phi = this -> angles[ num - 1] * __RAD__;
    target[0] = 1; target[4] = 0; target[8] = 0;
    target[1] = 0; target[5] = cos ( phi ); target[9] = -sin ( phi );
    target[2] = 0; target[6] = sin ( phi ); target[10] = cos ( phi );
}

void ArmDevice::rotateY(uint8_t num, float32_t* target)
{
    float32_t phi = ( this -> angles[ num - 1] ) * __RAD__;
    target[0] = cos ( phi ); target[4] = 0; target[8] = sin ( phi );
    target[1] = 0; target[5] = 1; target[9] = 0;
    target[2] =-sin ( phi ); target[6] = 0; target[10] = cos ( phi );
}

void ArmDevice::rotateZ(uint8_t num, float32_t* target)
{
    float32_t phi = ( this -> angles[ num - 1] ) * __RAD__ ;
    target[0] = cos ( phi ); target[4] = -sin ( phi ); target[8] = 0;
    target[1] = sin ( phi ); target[5] = cos ( phi ); target[9] = 0;
    target[2] = 0; target[6] = 0; target[10] = 1;
}

void ArmDevice::translateX(uint8_t num, float32_t* target )
{
    target[12] =  this -> translations[ num - 1 ];
}

void ArmDevice::translateY(uint8_t num, float32_t* target)
{
    target[13] =  this -> translations[ num - 1 ];
}

void ArmDevice::translateZ(uint8_t num, float32_t* target)
{
    target[14] = this -> translations[ num - 1 ];
}

void ArmDevice::calculate_end_effector(float32_t* target)
{   
    float32_t T1 [ 16 ] = { 0 };
    T1[15] = 1;
    translateZ(1, T1);
    rotateZ(1, T1);

    float32_t T2 [ 16 ] = { 0 };
    T2[15] = 1;
    translateX(2, T2);
    translateY(3, T2);
    translateZ(4, T2);
    rotateY(2, T2);
    //print_matrix(T2);

    //print_matrix(T2);

    float32_t T3 [ 16 ] = { 0 };
    T3[15] = 1;
    //translateX()
    translateY(5, T3);
    translateZ(6, T3);
    rotateY(3, T3);
    //print_matrix(T3);

    float32_t T4 [ 16 ] = { 0 };
    T4[15] = 1;
    translateX(7, T4);
    translateY(8, T4);
    rotateX(4, T4);

    float32_t T5 [ 16 ] = { 0 };
    T5[15] = 1;
    translateX(9, T5);
    rotateY(5, T5);

    float32_t T6 [ 16 ] = { 0 };
    T6[15] = 1;
    translateX(10, T6);
    rotateX(6, T6);

    float32_t T7 [ 16 ] = { 0 };
    T7[15] = 1;
    translateX(11, T7);
    rotateX(7, T7);

    //print_matrix(T7);

    float32_t step1[16];
    neon_multiply(T1, T2, step1);
    //print_matrix(step1);

    float32_t step2[16];
    neon_multiply(step1, T3, step2);
    //print_matrix(step2);

    float32_t step3[16];
    neon_multiply(step2, T4, step3);
    //print_matrix(step3);

    float32_t step4[16];
    neon_multiply(step3, T5, step4);
    //print_matrix(step4);
    float32_t step5[16];
    neon_multiply(step4, T6, step5);
    //print_matrix(step5);
    //float32_t step6[16];
    neon_multiply(step5, T7, target);

    //neon_multiply(step6, T7, target);
}


void ArmDevice::print_matrix(float32_t *M)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%f ", M[j * 4 + i]);
        }
        printf("\n");
    }
    printf("\n");
}

float32_t ArmDevice::servo_read(uint8_t id)
{
    if (((id > 6) || (id < 1)) == 1)
    {
        UnmappedError err;
        throw err;
    }
    
    id += 0x30;
    uint8_t buf[3] = {id, 0};
    uint16_t pos = write(this->bus, buf, 2);

    float32_t val;

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

float32_t *ArmDevice::servo_readall()
{
    float32_t* values = new float32_t[6];
    for (uint8_t i = 1; i <= 6; i++){

        values[i - 1] = this->servo_read(i);
        //std::cout << values[i - 1] << " ";
    }
    //std::cout << '\n';
    

    return values;
}

float32_t ArmDevice::servo_read_any(uint8_t id)
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

void ArmDevice::home_position()
{
    this -> toggleTorque(true);
    uint16_t angles[] = {90, 135, 45, 0, 90, 90};
    this -> servo_write6(angles, 1000);
    usleep(1000000);
    //toggleTorque(0);
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

void ArmDevice::bus_cleaner(uint8_t* dest, uint16_t time)
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