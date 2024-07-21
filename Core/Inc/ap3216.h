#ifndef INC_AP3216_H_
#define INC_AP3216_H_


// System Register Table
typedef enum
{
    SYS_REG_SYS_CONFIG = 0x00,    // Control of basic functions
    SYS_REG_INT_STATUS = 0x01,    // ALS and PS interrupt status output
    SYS_REG_INT_CLEAR_MANNER = 0x02,    // Auto/semi clear INT pin selector
    SYS_REG_IR_DATA_LOW = 0x0A,    // Lower byte for IR ADC channel output
    SYS_REG_IR_DATA_HIGH = 0x0B,    // Higher byte for IR ADC channel output
    SYS_REG_ALS_DATA_LOW = 0x0C,    // Lower byte for ALS ADC channel output
    SYS_REG_ALS_DATA_HIGH = 0x0D,    // Higher byte for ALS ADC channel output
    SYS_REG_PS_DATA_LOW = 0x0E,    // Lower byte for PS ADC channel output
    SYS_REG_PS_DATA_HIGH = 0x0F     // Higher byte for PS ADC channel output
} SystemRegisterTable;

// ALS Register Table
typedef enum
{
    ALS_REG_CONFIG = 0x10,    // Control of gain, conversion time of persist for ALS
    ALS_REG_CALIBRATION = 0x19,    // ALS window loss calibration
    ALS_REG_LOW_THRESHOLD_7_0 = 0x1A,    // Lower byte of ALS low threshold
    ALS_REG_LOW_THRESHOLD_15_8 = 0x1B,    // Higher byte of ALS low threshold
    ALS_REG_HIGH_THRESHOLD_7_0 = 0x1C,    // Lower byte of ALS high threshold
    ALS_REG_HIGH_THRESHOLD_15_8 = 0x1D     // Higher byte of ALS high threshold
} ALSRegisterTable;

// PS Register Table
typedef enum
{
    PS_REG_CONFIG = 0x20,    // Control of gain, integrated time and persist for PS
    PS_REG_LED_DRIVER = 0x21,    // Control of LED pulses number and driver current
    PS_REG_INT_FORM = 0x22,    // Interrupt algorithms style select of PS
    PS_REG_MEAN_TIME = 0x23,    // PS average time selector
    PS_REG_LED_WAITING_TIME = 0x24,    // Control PS LED waiting time
    PS_REG_CALIBRATION_L = 0x28,    // Offset value to eliminate cross talk
    PS_REG_CALIBRATION_H = 0x29,    // Offset value to eliminate cross talk
    PS_REG_LOW_THRESHOLD_2_0 = 0x2A,    // Lower byte of PS low threshold
    PS_REG_LOW_THRESHOLD_10_3 = 0x2B,    // Higher byte of PS low threshold
    PS_REG_HIGH_THRESHOLD_2_0 = 0x2C,    // Lower byte of PS high threshold
    PS_REG_HIGH_THRESHOLD_10_3 = 0x2D     // Higher byte of PS high threshold
} PSRegisterTable;

#define AP3216_ID           0x1E
#define AP3216_READ         0x01
#define AP3216_WRITE        0x00
#define AP3216_READ_ADD     (AP3216_ID << 1 | AP3216_READ)
#define AP3216_WRITE_ADD    (AP3216_ID << 1 | AP3216_WRITE)

#define SW_RESET            0x04
#define ALS_PS_IR_ACTIVE    0x03

#define I2C_WRITE_DATA_SIZE 2
#define ADDRESS_DATA_INDEX  0
#define VALUE_DATA_INDEX    1

#define I2C_TIMEOUT 200


void AP3216_Init(I2C_HandleTypeDef *hi2c);


#endif /* INC_AP3216_H_ */
