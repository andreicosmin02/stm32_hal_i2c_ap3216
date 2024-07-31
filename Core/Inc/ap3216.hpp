#ifndef INC_AP3216_HPP_
#define INC_AP3216_HPP_

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

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
    PS_REG_LOW_THRESHOLD_1_0 = 0x2A,    // Lower byte of PS low threshold
    PS_REG_LOW_THRESHOLD_9_2 = 0x2B,    // Higher byte of PS low threshold
    PS_REG_HIGH_THRESHOLD_1_0 = 0x2C,    // Lower byte of PS high threshold
    PS_REG_HIGH_THRESHOLD_9_2 = 0x2D     // Higher byte of PS high threshold
} PSRegisterTable;

// System configuration
typedef enum
{
    POWER_DOWN = 0x00,
    ALS_FUNCTION_ACTIVE,
    PS_IR_FUNCTION_ACTIVE,
    ALS_PS_IR_FUNCTIONS_ACTIVE,
    SW_RESET,
    ALS_FUNCTION_ONCE,
    PS_IR_FUNCTION_ONCE,
    ALS_PS_IR_FUNCTIONS_ONCE
} System_ConfTypeDef;


// ALS Range
typedef enum
{
    RANGE_20661 = 0,
    RANGE_5162 = 0x10,
    RANGE_1291 = 0x20,
    RANGE_323 = 0x30
} ALS_RangeTypeDef;

// ALS Range factor
#define FACTOR_RANGE_20661  0.35
#define FACTOR_RANGE_5162   0.0788
#define FACTOR_RANGE_1291   0.0197
#define FACTOR_RANGE_323    0.0049

// ALS Persist
typedef enum
{
    CONVERSION_TIME_1 = 0x00,
    CONVERSION_TIME_4 = 0x01,
    CONVERSION_TIME_8 = 0x02,
    CONVERSION_TIME_12 = 0x03,
    CONVERSION_TIME_16 = 0x04,
    CONVERSION_TIME_60 = 0x0F
} ALS_PersistTypeDef;

// PS Gain
typedef enum
{
    PS_GAIN_1 = 0x00,
    PS_GAIN_2 = 0x01,
    PS_GAIN_4 = 0x02,
    PS_GAIN_8 = 0x03
} PS_GainTypeDef;

// PS LED Pulse
typedef enum
{
    PULSE_0 = 0x00,
    PULSE_1 = 0x01,
    PULSE_2 = 0x02,
    PULSE_3 = 0x03
} PS_LED_PulseTypeDef;

// PS LED Driver Ratio
typedef enum
{
    DRIVER_RATIO_16 = 0x00,
    DRIVER_RATIO_33 = 0x01,
    DRIVER_RATIO_66 = 0x02,
    DRIVER_RATIO_100 = 0x03
} PS_LED_DriverRatioTypeDef;

// PS INT Mode
typedef enum
{
    ZONE_TYPE_MODE = 0x00,
    HYSTERESIS_TYPE_MODE = 0x01
} PS_INT_ModeTypeDef;

// PS Mean Time
typedef enum
{
    MEAN_TIME_12 = 0x00,
    MEAN_TIME_25 = 0x01,
    MEAN_TIME_37 = 0x02,
    MEAN_TIME_50 = 0x03
} PS_MeanTimeTypeDef;

#define AP3216_ID           0x1E
#define AP3216_READ         0x01
#define AP3216_WRITE        0x00
#define AP3216_READ_ADD     (AP3216_ID << 1 | AP3216_READ)
#define AP3216_WRITE_ADD    (AP3216_ID << 1 | AP3216_WRITE)

#define SW_RESET            0x04
#define ALS_PS_IR_ACTIVE    0x03

#define I2C_WRITE_DATA_SIZE     2
#define ADDRESS_DATA_INDEX      0
#define VALUE_DATA_INDEX        1
#define I2C_READ_DATA_SIZE      2
#define I2C_READ_SINGLE_SIZE    1
#define I2C_READ_MEM_SIZE       1

#define I2C_TIMEOUT 200

#define ALS_DYNAMIC_RANGE_MASK  0xCF
#define ALS_PERSIST_MASK        0xF0
#define ALS_THRESHOLD_MASK      0xFF
#define ALS_REG_SIZE            0x08
#define PS_TIME_SELECT_MASK     0x0F
#define PS_TIME_SELECT_BIT_0    0x04
#define PS_GAIN_MASK            0xF3
#define PS_GAIN_BIT_0           0x02
#define PS_LED_PULSE_MASK       0xCF
#define PS_LED_PULSE_BIT_0      0x04
#define PS_LED_DRIVER_RATIO_MASK    0xFC
#define PS_INT_MODE_MASK        0xFE
#define PS_MEAN_TIME_MASK       0xFC
#define PS_LED_WAITING_MASK     0xC0
#define PS_LED_CALIBRATION_L_MASK   0xFE
#define PS_LED_CALIBRATION_L_BIT    0x01
#define PS_LED_CALIBRATION_H_MASK   0xFF

#define PS_LED_LOW_THRESHOLD_L_MASK 0xFC
#define PS_LED_LOW_THRESHOLD_L_BIT  0x02
#define PS_LED_LOW_THRESHOLD_H_MASK 0xFF

#define PS_LED_HIGH_THRESHOLD_L_MASK 0xFC
#define PS_LED_HIGH_THRESHOLD_L_BIT  0x02
#define PS_LED_HIGH_THRESHOLD_H_MASK 0xFF

#define DATA_HIGH_INDEX     0x00
#define DATA_LOW_INDEX      0x01

#define IR_DATA_VALID_MASK      0x80
#define IR_DATA_HIGH_MASK       0x03
#define IR_DATA_LOW_SIZE        0x02

#define ALS_DATA_LOW_SIZE       0x08

#define PS_DATA_OBJECT_DETECT_MASK  0x80
#define PS_DATA_IR_OVERFLOW_MASK    0x40
#define PS_DATA_LOW_MASK            0x0F
#define PS_DATA_LOW_SIZE            0x04

#define AP3216_ALS_CALIBRATION_DEFAULT_VALUE    0x40
#define AP3216_PS_TIME_SELECT_DEFAULT_VALUE     0x00
#define AP3216_PS_LED_WAITING_DEFAULT_VALUE     0x00


#ifdef __cplusplus
}


struct ap3216_values
{
    double ambient_light;
    uint16_t proximity;
    uint16_t IR_light;
    bool object_is_near;
    bool ir_valid;
} ;

struct AP3216_ConfigTypeDef
{
    System_ConfTypeDef System_Conf = POWER_DOWN;
    ALS_RangeTypeDef ALS_range = RANGE_20661;
    double ALS_dynamic_range_factor = FACTOR_RANGE_20661;
    ALS_PersistTypeDef ALS_Persist = CONVERSION_TIME_1;
    uint8_t ALS_calibration = AP3216_ALS_CALIBRATION_DEFAULT_VALUE;
    uint16_t ALS_low_threshold;
    uint16_t ALS_high_threshold;
    uint8_t PS_Time_Select = AP3216_PS_TIME_SELECT_DEFAULT_VALUE;
    PS_GainTypeDef PS_Gain = PS_GAIN_2;
    PS_LED_PulseTypeDef PS_LED_Pulse = PULSE_1;
    PS_LED_DriverRatioTypeDef PS_LED_DriverRatio = DRIVER_RATIO_100;
    PS_INT_ModeTypeDef PS_INT_Mode = HYSTERESIS_TYPE_MODE;
    PS_MeanTimeTypeDef PS_MeanTime = MEAN_TIME_12;
    uint8_t PS_LED_Waiting = AP3216_PS_LED_WAITING_DEFAULT_VALUE;
    uint16_t PS_LED_Calibration;
    uint16_t PS_LED_low_threshold;
    uint16_t PS_LED_high_threshold;
} ;


class AP3216
{
private:
    ap3216_values values;
    I2C_HandleTypeDef *hi2c;
    AP3216_ConfigTypeDef *AP3216_config;
public:

    void AP3216_Init(I2C_HandleTypeDef *hi2c, AP3216_ConfigTypeDef *AP3216_config);

    void AP3216_SystemConfiguration(System_ConfTypeDef configuration);

    void AP3216_ALS_DynamicRange(ALS_RangeTypeDef range);

    void AP3216_ALS_Persist(ALS_PersistTypeDef persist);

    void AP3216_ALS_Calibration(uint8_t calibration);

    void AP3216_ALS_Threshold(uint16_t low_threshold, uint16_t high_threshold);

    void AP3216_PS_TimeSelect(uint8_t time_select);

    void AP3216_PS_Gain(PS_GainTypeDef gain);

    void AP3216_PS_LED_Pulse(PS_LED_PulseTypeDef pulse);

    void AP3216_LED_DriverRatio(PS_LED_DriverRatioTypeDef driver_ratio);

    void AP3216_PS_INT_Mode(PS_INT_ModeTypeDef int_mode);

    void AP3216_PS_MeanTime(PS_MeanTimeTypeDef mean_time);

    void AP3216_PS_LED_Waiting(uint8_t led_waiting);

    void AP3216_PS_LED_Calibration(uint16_t calibration);

    void AP3216_PS_LED_Threshold(uint16_t low_threshold, uint16_t high_threshold);

    void AP3216_Values(ap3216_values *sensor_values);

    void AP3216_Write(uint8_t i2c_address, uint8_t value_to_write);

    void AP3216_Read(uint8_t i2c_address, uint8_t *data, uint8_t data_size);

    bool AP3216_GetIRValid();

    uint16_t AP3216_GetIRLight();

    double AP3216_GetAmbientLight();

    bool AP3216_GetObjectNear();

    uint16_t AP3216_GetProximity();

    void TransmitSensorValues(UART_HandleTypeDef *huart, ap3216_values *sensor_values);
};
#endif

#endif /* INC_AP3216_HPP_ */
