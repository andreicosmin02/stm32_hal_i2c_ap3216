#include "ap3216.hpp"

void AP3216::AP3216_Init(I2C_HandleTypeDef *I2C_Handle, AP3216_ConfigTypeDef *AP3216)
{
    hi2c = I2C_Handle;
    AP3216_config = AP3216;

    AP3216_SystemConfiguration(AP3216_config->System_Conf);
    AP3216_ALS_DynamicRange(AP3216_config->ALS_range);
    AP3216_ALS_Persist(AP3216_config->ALS_Persist);
    AP3216_ALS_Calibration(AP3216_config->ALS_calibration);
    AP3216_ALS_Threshold(AP3216_config->ALS_low_threshold, AP3216_config->ALS_high_threshold);
    AP3216_PS_TimeSelect(AP3216_config->PS_Time_Select);
    AP3216_PS_Gain(AP3216_config->PS_Gain);
    AP3216_PS_LED_Pulse(AP3216_config->PS_LED_Pulse);
    AP3216_LED_DriverRatio(AP3216_config->PS_LED_DriverRatio);
    AP3216_PS_INT_Mode(AP3216_config->PS_INT_Mode);
    AP3216_PS_MeanTime(AP3216_config->PS_MeanTime);
    AP3216_PS_LED_Waiting(AP3216_config->PS_LED_Waiting);
    AP3216_PS_LED_Calibration(AP3216_config->PS_LED_Calibration);
    AP3216_PS_LED_Threshold(AP3216_config->PS_LED_low_threshold, AP3216_config->PS_LED_high_threshold);
}

void AP3216::AP3216_SystemConfiguration(System_ConfTypeDef configuration)
{
    AP3216_Write(SYS_REG_SYS_CONFIG, configuration);
}

void AP3216::AP3216_ALS_DynamicRange(ALS_RangeTypeDef range)
{
    switch (range)
    {
        case RANGE_20661:
            AP3216_config->ALS_dynamic_range_factor = FACTOR_RANGE_20661;
            break;
        case RANGE_5162:
            AP3216_config->ALS_dynamic_range_factor = FACTOR_RANGE_5162;
            break;
        case RANGE_1291:
            AP3216_config->ALS_dynamic_range_factor = FACTOR_RANGE_1291;
            break;
        case RANGE_323:
            AP3216_config->ALS_dynamic_range_factor = FACTOR_RANGE_323;
            break;
    }
    uint8_t als_conf;
    AP3216_Read(ALS_REG_CONFIG, &als_conf, I2C_READ_SINGLE_SIZE);
    als_conf &= ALS_DYNAMIC_RANGE_MASK;
    als_conf |= range;
    AP3216_Write(ALS_REG_CONFIG, als_conf);
}

void AP3216::AP3216_ALS_Persist(ALS_PersistTypeDef persist)
{
    uint8_t als_conf;
    AP3216_Read(ALS_REG_CONFIG, &als_conf, I2C_READ_SINGLE_SIZE);
    als_conf &= ALS_PERSIST_MASK;
    als_conf |= persist;
    AP3216_Write(ALS_REG_CONFIG, als_conf);
}

void AP3216::AP3216_ALS_Calibration(uint8_t calibration)
{
    AP3216_Write(ALS_REG_CALIBRATION, calibration);
}

void AP3216::AP3216_ALS_Threshold(uint16_t low_threshold, uint16_t high_threshold)
{
    uint8_t low_threshold_l = low_threshold & ALS_THRESHOLD_MASK;
    uint8_t low_threshold_h = low_threshold >> ALS_REG_SIZE;
    uint8_t high_threshold_l = high_threshold & ALS_THRESHOLD_MASK;
    uint8_t high_threshold_h = high_threshold >> ALS_REG_SIZE;
    AP3216_Write(ALS_REG_LOW_THRESHOLD_7_0, low_threshold_l);
    AP3216_Write(ALS_REG_LOW_THRESHOLD_15_8, low_threshold_h);
    AP3216_Write(ALS_REG_HIGH_THRESHOLD_7_0, high_threshold_l);
    AP3216_Write(ALS_REG_HIGH_THRESHOLD_15_8, high_threshold_h);
}

void AP3216::AP3216_PS_TimeSelect(uint8_t time_select)
{
    uint8_t ps_conf;
    AP3216_Read(PS_REG_CONFIG, &ps_conf, I2C_READ_SINGLE_SIZE);
    ps_conf &= PS_TIME_SELECT_MASK;
    ps_conf |= time_select << PS_TIME_SELECT_BIT_0;
    AP3216_Write(PS_REG_CONFIG, ps_conf);
}

void AP3216::AP3216_PS_Gain(PS_GainTypeDef gain)
{
    uint8_t ps_conf;
    AP3216_Read(PS_REG_CONFIG, &ps_conf, I2C_READ_SINGLE_SIZE);
    ps_conf &= PS_GAIN_MASK;
    ps_conf |= gain << PS_GAIN_BIT_0;
    AP3216_Write(PS_REG_CONFIG, ps_conf);
}

void AP3216::AP3216_PS_LED_Pulse(PS_LED_PulseTypeDef pulse)
{
    uint8_t ps_led_control;
    AP3216_Read(PS_REG_LED_DRIVER, &ps_led_control, I2C_READ_SINGLE_SIZE);
    ps_led_control &= PS_LED_PULSE_MASK;
    ps_led_control |= pulse << PS_LED_PULSE_BIT_0;
    AP3216_Write(PS_REG_LED_DRIVER, ps_led_control);
}

void AP3216::AP3216_LED_DriverRatio(PS_LED_DriverRatioTypeDef driver_ratio)
{
    uint8_t ps_led_control;
    AP3216_Read(PS_REG_LED_DRIVER, &ps_led_control, I2C_READ_SINGLE_SIZE);
    ps_led_control &= PS_LED_DRIVER_RATIO_MASK;
    ps_led_control |= driver_ratio;
    AP3216_Write(PS_REG_LED_DRIVER, ps_led_control);
}

void AP3216::AP3216_PS_INT_Mode(PS_INT_ModeTypeDef int_mode)
{
    uint8_t ps_int_mode;
    AP3216_Read(PS_REG_INT_FORM, &ps_int_mode, I2C_READ_SINGLE_SIZE);
    ps_int_mode &= PS_INT_MODE_MASK;
    ps_int_mode |= int_mode;
    AP3216_Write(PS_REG_INT_FORM, ps_int_mode);
}

void AP3216::AP3216_PS_MeanTime(PS_MeanTimeTypeDef mean_time)
{
    uint8_t ps_mean_time;
    AP3216_Read(PS_REG_MEAN_TIME, &ps_mean_time, I2C_READ_SINGLE_SIZE);
    ps_mean_time &= PS_MEAN_TIME_MASK;
    ps_mean_time |= mean_time;
    AP3216_Write(PS_REG_MEAN_TIME, ps_mean_time);
}

void AP3216::AP3216_PS_LED_Waiting(uint8_t led_waiting)
{
    uint8_t ps_led_waiting;
    AP3216_Read(PS_REG_LED_WAITING_TIME, &ps_led_waiting, I2C_READ_SINGLE_SIZE);
    ps_led_waiting &= PS_LED_WAITING_MASK;
    ps_led_waiting |= led_waiting;
    AP3216_Write(PS_REG_LED_WAITING_TIME, ps_led_waiting);
}

void AP3216::AP3216_PS_LED_Calibration(uint16_t calibration)
{
    uint8_t calibration_low;
    uint8_t calibration_high = (calibration >> PS_LED_CALIBRATION_L_BIT) & PS_LED_CALIBRATION_H_MASK;

    AP3216_Read(PS_REG_CALIBRATION_L, &calibration_low, I2C_READ_SINGLE_SIZE);
    calibration_low &= PS_LED_CALIBRATION_L_MASK;
    calibration_low |= (calibration & (~PS_LED_CALIBRATION_L_MASK));

    AP3216_Write(PS_REG_CALIBRATION_L, calibration_low);
    AP3216_Write(PS_REG_CALIBRATION_H, calibration_high);
}

void AP3216::AP3216_PS_LED_Threshold(uint16_t low_threshold, uint16_t high_threshold)
{
    // Low Threshold configuration
    uint8_t low_threshold_l;
    uint8_t low_threshold_h = (low_threshold >> PS_LED_LOW_THRESHOLD_L_BIT) & PS_LED_LOW_THRESHOLD_H_MASK;

    AP3216_Read(PS_REG_LOW_THRESHOLD_1_0, &low_threshold_l, I2C_READ_SINGLE_SIZE);
    low_threshold_l &= PS_LED_LOW_THRESHOLD_L_MASK;
    low_threshold_l |= (low_threshold & (~PS_LED_LOW_THRESHOLD_L_MASK));
    AP3216_Write(PS_REG_LOW_THRESHOLD_1_0, low_threshold_l);
    AP3216_Write(PS_REG_LOW_THRESHOLD_9_2, low_threshold_h);

    // High Threshold configuration
    uint8_t high_threshold_l;
    uint8_t high_threshold_h = (high_threshold >> PS_LED_HIGH_THRESHOLD_L_BIT) & PS_LED_HIGH_THRESHOLD_H_MASK;

    AP3216_Read(PS_REG_HIGH_THRESHOLD_1_0, &high_threshold_l, I2C_READ_SINGLE_SIZE);
    high_threshold_l &= PS_LED_HIGH_THRESHOLD_L_MASK;
    high_threshold_l |= (high_threshold & (~PS_LED_HIGH_THRESHOLD_L_MASK));
    AP3216_Write(PS_REG_HIGH_THRESHOLD_1_0, high_threshold_l);
    AP3216_Write(PS_REG_HIGH_THRESHOLD_9_2, high_threshold_h);
}

void AP3216::AP3216_Values(ap3216_values *sensor_values)
{

}

void AP3216::AP3216_Write(uint8_t i2c_address, uint8_t value_to_write)
{
    uint8_t data[I2C_WRITE_DATA_SIZE] = {i2c_address, value_to_write};
    if (HAL_I2C_Master_Transmit(
            hi2c,
            AP3216_WRITE_ADD,
            data,
            I2C_WRITE_DATA_SIZE,
            I2C_TIMEOUT
    )
        == HAL_ERROR)
    {
        Error_Handler();
    }
}

void AP3216::AP3216_Read(uint8_t i2c_address, uint8_t *data, uint8_t data_size)
{
    if (HAL_I2C_Mem_Read(
            hi2c,
            AP3216_READ_ADD,
            i2c_address,
            I2C_READ_MEM_SIZE,
            data,
            data_size,
            I2C_TIMEOUT
    )
        == HAL_ERROR)
    {
        Error_Handler();
    }
}

bool AP3216::AP3216_GetIRValid()
{
    uint8_t IR_data[I2C_READ_DATA_SIZE];
    // IR data reading
    AP3216_Read(SYS_REG_IR_DATA_LOW, IR_data, I2C_READ_DATA_SIZE);
    bool ir_data_valid = !(IR_data[DATA_HIGH_INDEX] & IR_DATA_VALID_MASK);

    return ir_data_valid;
}

uint16_t AP3216::AP3216_GetIRLight()
{
    uint8_t IR_data[I2C_READ_DATA_SIZE];

    // IR data reading
    AP3216_Read(SYS_REG_IR_DATA_LOW, IR_data, I2C_READ_DATA_SIZE);
    uint8_t ir_data_high = IR_data[DATA_HIGH_INDEX] & IR_DATA_HIGH_MASK;
    uint8_t ir_data_low = IR_data[DATA_LOW_INDEX];

    uint16_t IR_light = (ir_data_high << IR_DATA_LOW_SIZE) | ir_data_low;
    return IR_light;
}

double AP3216::AP3216_GetAmbientLight()
{
    uint8_t ALS_data[I2C_READ_DATA_SIZE];

    // ALS data reading
    AP3216_Read(SYS_REG_ALS_DATA_LOW, ALS_data, I2C_READ_DATA_SIZE);
    uint16_t als_data = (ALS_data[DATA_HIGH_INDEX] << ALS_DATA_LOW_SIZE) | ALS_data[DATA_LOW_INDEX];

    double ambient_light = (double) als_data * AP3216_config->ALS_dynamic_range_factor;
    return ambient_light;
}

bool AP3216::AP3216_GetObjectNear()
{
    uint8_t PS_data[I2C_READ_DATA_SIZE];

    // PS data reading
    // PS Low
    AP3216_Read(SYS_REG_PS_DATA_LOW, &PS_data[DATA_LOW_INDEX], I2C_READ_SINGLE_SIZE);
    bool object_near = PS_data[DATA_LOW_INDEX] & PS_DATA_OBJECT_DETECT_MASK;

    return object_near;
}

uint16_t AP3216::AP3216_GetProximity()
{
    uint8_t PS_data[I2C_READ_DATA_SIZE];

    // PS data reading
    // PS Low
    AP3216_Read(SYS_REG_PS_DATA_LOW, &PS_data[DATA_LOW_INDEX], I2C_READ_SINGLE_SIZE);
    uint8_t ps_data_low = PS_data[DATA_LOW_INDEX] & PS_DATA_LOW_MASK;

    // PS High
    AP3216_Read(SYS_REG_PS_DATA_HIGH, &PS_data[DATA_HIGH_INDEX], I2C_READ_SINGLE_SIZE);
    uint8_t ps_data_high = PS_data[DATA_HIGH_INDEX];

    uint16_t proximity = (ps_data_high << PS_DATA_LOW_SIZE) | ps_data_low;
    return proximity;
}

void AP3216::TransmitSensorValues(UART_HandleTypeDef *huart, ap3216_values *sensor_values)
{
    char buffer[200]; // Allocate a buffer large enough to hold the formatted string
    int length;

    // Format the values into the buffer
    length = snprintf(
            buffer,
            sizeof(buffer),
            "Ambient Light: %.2f\r\nProximity: %u\r\nIR Light: %u\r\nObject is Near: %s\r\nIR Valid: %s\r\n\r\n",
            sensor_values->ambient_light,
            sensor_values->proximity,
            sensor_values->IR_light,
            sensor_values->object_is_near ? "True" : "False",
            sensor_values->ir_valid ? "True" : "False"
    );

    HAL_UART_Transmit(huart, (uint8_t *) buffer, length, HAL_MAX_DELAY);
}



