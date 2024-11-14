#include "ltc2497.h"

extern I2C_HandleTypeDef hi2c1; // Ensure this I2C handle is defined and initialized in your main code

/* Reads data from the LTC2497 ADC */
int8_t LTC2497_Read(uint8_t i2c_address, uint8_t adc_command, int32_t *adc_code, uint16_t timeout)
{
    uint8_t data[3];
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();

    /* Send the command byte to start conversion */
    status = HAL_I2C_Master_Transmit(&hi2c1, (i2c_address << 1), &adc_command, 1, timeout);
    if (status != HAL_OK)
    {
        return 1; /* Error in sending command */
    }

    /* Wait for conversion to complete */
    while (1)
    {
        /* Attempt to read data */
        status = HAL_I2C_Master_Receive(&hi2c1, (i2c_address << 1), data, 3, timeout); // Read 3 bytes

        if (status == HAL_OK)
        {
            break; /* Data received successfully */
        }
        if ((HAL_GetTick() - tickstart) > timeout)
        {
            return 1; /* Timeout */
        }
        HAL_Delay(1);
    }

    /* Combine the three bytes into a single 24-bit value */
    uint32_t raw_code = ((uint32_t)data[0] << 16) |
                        ((uint32_t)data[1] << 8) |
                        data[2];

    /* Extract SIG and MSB bits */
    uint8_t sig = (raw_code >> 23) & 0x01;
    uint8_t msb = (raw_code >> 22) & 0x01;

    /* Extract the 16-bit conversion result */
    uint16_t conversion_result = ((data[0] & 0x3F) << 10) | (data[1] << 2) | (data[2] >> 6);


    /* Handle Over-Range and Under-Range Conditions */
    if (sig && msb)
    {
        /* VIN >= FS (Over-Range Positive) */
        *adc_code = 2147483647; // Maximum positive value for 32-bit
    }
    else if (!sig && !msb)
    {
        /* VIN < -FS (Under-Range Negative) */
        *adc_code = -2147483648; // Maximum negative value for 16-bit
    }
    else if (sig && !msb)
    {
    	/* Treat as unsigned positive */
    	*adc_code = (int32_t)conversion_result;
    }
    else // (!sig && msb)
    {
        /* Normal Negative Range */
        /* Treat as signed two's complement */
        *adc_code = (int32_t)((int16_t)conversion_result);
    }

    return 0; /* Success */
}

/* Converts ADC code to voltage */
float LTC2497_CodeToVoltage(int32_t adc_code, float vref)
{
    float voltage;

    /* The full-scale range is FS = 0.5 * Vref */
    /* Voltage calculation based on two's complement */
    voltage = ((float)adc_code / 33497.03184f);

    return voltage;
}
