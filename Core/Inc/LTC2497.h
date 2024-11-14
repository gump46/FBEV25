#ifndef LTC2497_H
#define LTC2497_H

#include "stm32wbxx_hal.h" // Replace 'stm32fxxx' with your specific STM32 series, e.g., 'stm32f4xx'

/* I2C address for LTC2497
   The I2C address depends on the states of the CA0, CA1, and CA2 pins.
   Adjust the address below according to your hardware configuration.
*/
#define LTC2497_I2C_ADDRESS 0x76 // Example address; please adjust as needed

/* Oversampling ratio commands */
#define LTC2497_OSR_64         0x10
#define LTC2497_OSR_128        0x20
#define LTC2497_OSR_256        0x30
#define LTC2497_OSR_512        0x40
#define LTC2497_OSR_1024       0x50
#define LTC2497_OSR_2048       0x60
#define LTC2497_OSR_4096       0x70
#define LTC2497_OSR_8192       0x80
#define LTC2497_OSR_16384      0x90
#define LTC2497_OSR_32768      0xF0

/* Single-ended channel selection commands */
#define LTC2497_CH0            0xB0
#define LTC2497_CH1            0xB8
#define LTC2497_CH2            0xB1
#define LTC2497_CH3            0xB9
#define LTC2497_CH4            0xB2
#define LTC2497_CH5            0xBA
#define LTC2497_CH6            0xB3
#define LTC2497_CH7            0xBB
#define LTC2497_CH8            0xB4
#define LTC2497_CH9            0xBC
#define LTC2497_CH10           0xB5
#define LTC2497_CH11           0xBD
#define LTC2497_CH12           0xB6
#define LTC2497_CH13           0xBE
#define LTC2497_CH14           0xB7
#define LTC2497_CH15           0xBF

/* Differential channel selection commands */
#define LTC2497_CH_P0_N1       0xA0
#define LTC2497_CH_P1_N0       0xA8
#define LTC2497_CH_P2_N3       0xA1
#define LTC2497_CH_P3_N2       0xA9
#define LTC2497_CH_P4_N5       0xA2
#define LTC2497_CH_P5_N4       0xAA
#define LTC2497_CH_P6_N7       0xA3
#define LTC2497_CH_P7_N6       0xAB
#define LTC2497_CH_P8_N9       0xA4
#define LTC2497_CH_P9_N8       0xAC
#define LTC2497_CH_P10_N11     0xA5
#define LTC2497_CH_P11_N10     0xAD
#define LTC2497_CH_P12_N13     0xA6
#define LTC2497_CH_P13_N12     0xAE
#define LTC2497_CH_P14_N15     0xA7
#define LTC2497_CH_P15_N14     0xAF

/* Function prototypes */
int8_t LTC2497_Read(uint8_t i2c_address, uint8_t adc_command, int32_t *adc_code, uint16_t timeout);
float LTC2497_CodeToVoltage(int32_t adc_code, float vref);

#endif /* LTC2497_H */
