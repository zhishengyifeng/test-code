#ifndef IST8310_H
#define IST8310_H

#include <stdint.h>
#include "struct_typedef.h"

typedef struct ist8310_raw_data_t {
    float x;
    float y;
    float z;
} ist8310_raw_data_t;

typedef enum ist8310_error_e {
    IST8310_NO_ERROR = 0x00,
    MEG_ID_ERROR = 0x01,
} ist8310_error_e;

typedef struct ist8310_data_t {
    uint8_t chip_id;
    fp32 Mag[3];
    ist8310_error_e meg_error;
} ist8310_data_t;

/*-----������uTת��-----*/
#define MAG_SEN 0.3f

/*-----I2C�ӿڶ���-----*/
#define IST8310_Address 0x0E << 1
#define IST8310_I2C hi2c3

/*-----GPIO�ڶ���-----*/
#define IST8310_GPIOx GPIOG
#define IST8310_GPIOp GPIO_Pin_6

uint8_t IST8310_INIT(void);

// ������ȡ����
void IST8310_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t IST8310_ReadReg(uint8_t RegAddress);

// ���ܺ���
void ReadIST8310Data(fp32 Meg[3]);

// У�麯��
ist8310_error_e VerifyMegId(uint8_t* id);

#endif
