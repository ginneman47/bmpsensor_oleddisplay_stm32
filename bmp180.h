/*
 * bmp180.h
 *
 *  Created on: May 9, 2024
 *      Author: Joseph
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_


//#include "stm32f1xx_hal.h"
// oversampling definitions - pressure
#define OSRS_Skipped 0x00
#define OSRS_1 0x01
#define OSRS_2 0x02
#define OSRS_4 0x03
#define OSRS_8 0x04
#define OSRS_16 0x05


//extern I2C_HandleTypeDef hi2cl;// not needed

// mode definitions

#define MODE_SLEEP 0x00
#define MODE_FORCED 0x01
#define MODE_NORMAL 0x03

// IIR filter options
#define IIR_FILTER_OFF 0x00
#define IIR_FILTER_2 0x01
#define IIR_FILTER_4 0x02
#define IIR_FILTER_8 0x03
#define IIR_FILTER_16 0x04


//Register definitions
#define ID_REG 0xD0
#define RESET_REG 0xE0
#define CTRL_HUM_REG 0xF2
#define STATUS_REG 0xF3
#define CTRL_MEAS_REG 0xF4
#define CONFIG_REG 0xF5
#define PRESS_REG_MSB 0xF7
#define PRESS_REG_LSB 0xF8
#define PRESS_REG_XLSB 0xF9
#define TEMP_MSB 0xFA
#define TEMP_LSB 0xFB
#define TEMP_XLSB 0xFC
#define HUM_MSB 0xFD
#define HUM_LSB 0xFE


//standby time definitions
#define T_SB_P5 0x00
#define T_SB_62P5 0x01
#define T_SB_125 0x02
#define T_SB_250 0x03
#define T_SB_500 0x04
#define T_SB_1000 0x05
#define T_SB_10 0x06
#define T_SB_20 0x07


typedef long unsigned int BMP180_U32_t;
int BMP_Config(uint8_t osrs_t, uint8_t osrs_h, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter);
void TrimRead(void);
void BMP_WakeUP(void);
void BMP_Measure(void);
void BME_Read_Raw(void);
uint32_t BME280_compensate_P_int64(int32_t adc_P);
int32_t BME280_compensate_T_int32(int32_t adc_T);
uint32_t BME280_compensate_H_int32(int32_t adc_H);

#endif /* INC_BMP180_H_ */
