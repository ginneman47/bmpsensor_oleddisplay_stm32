 /*
 * bmp180.c
 *
 *  Created on: May 9, 2024
 *      Author: Joseph
 */
#include "stm32f1xx_hal.h"

#include "math.h"
#include "bmp180.h"

//#include "globaldefinitions.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
//#define BMP180_I2C &hi2c1;
#define BMP180_ADDRESS 0xEC;
#define SUPPORT_64BIT 1
extern float Temperature, Pressure, Humidity;
uint8_t chipId;
uint8_t TrimPara[36];
int64_t pRaw, tRaw, hRaw;
int32_t t_fine;

typedef long unsigned int BMP180_U32_t;
//unsigned short dig_T1;
//signed short dig_T2;
//signed short dig_T3;
//unsigned short dig_P1;
//signed short dig_P2;
//signed short dig_P3;
//signed short dig_P4;
//signed short dig_P5;
//signed short dig_P6;
//signed short dig_P7;
//signed short dig_P8;
//signed short dig_P9;
//unsigned char dig_H1;
//signed short dig_H2;
//unsigned char dig_H3;
//signed short dig_H4;
//signed short dig_H5;
//signed char dig_H6;

int16_t dig_T1;
int16_t dig_T2,dig_T3,dig_P1,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9,dig_H1,dig_H2,dig_H3,dig_H4,dig_H5,dig_H6;

//uncompensated pressure variables
signed long UT;
signed long UP;


//BMP180_U32_t tfine;
void TrimRead(void)
{
	uint8_t trimdata[32];
	HAL_I2C_Mem_Read(&hi2c1,0xEC,0x88,1,trimdata,25,HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1,0xEC,0xE1,1,(uint8_t *)trimdata+25,7,HAL_MAX_DELAY);

	//dig_T1 = (trimdata[1]<<8) | trimdata[0];
	dig_T1 = trimdata[1];
	dig_T1 = dig_T1<<8 | trimdata[0];
	//dig_T2 = (trimdata[3]<<8) | trimdata[2];
	dig_T2 = trimdata[3];
	dig_T2 = dig_T2<<8 | trimdata[2];
	//dig_T3 = (trimdata[5]<<8) | trimdata[4];
	dig_T3 = trimdata[5];
	dig_T3 = dig_T3<<8 | trimdata[4];
	//dig_P1 = (trimdata[7]<<8) | trimdata[6];
	dig_P1 = trimdata[7];
	dig_P1 = dig_P1<<8 | trimdata[6];
	//dig_P2 = (trimdata[9]<<8) | trimdata[8];
	dig_P2 = trimdata[9];
	dig_P2 = dig_P2<<8 | trimdata[8];
	//dig_P3 = (trimdata[11]<<8) | trimdata[10];
	dig_P3 = trimdata[11];
	dig_P3 = dig_P3<<8 | trimdata[10];
	//dig_P4 = (trimdata[13]<<8) | trimdata[12];
	dig_P4 = trimdata[13];
	dig_P4 = dig_P4<<8 | trimdata[12];
	//dig_P5 = (trimdata[15]<<8) | trimdata[14];
	dig_P5 = trimdata[15];
	dig_P5 = dig_P5<<8 | trimdata[14];
	//dig_P6 = (trimdata[17]<<8) | trimdata[16];
	dig_P6 = trimdata[17];
	dig_P6 = dig_P6<<8 | trimdata[16];
	//dig_P7 = (trimdata[19]<<8) | trimdata[18];
	dig_P7 = trimdata[19];
	dig_P7 = dig_P7<<8 | trimdata[18];
	//dig_P8 = (trimdata[21]<<8) | trimdata[20];
	dig_P8 = trimdata[21];
	dig_P8 = dig_P8<<8 | trimdata[20];
	//dig_P9 = (trimdata[23]<<8) | trimdata[22];
	dig_P9 = trimdata[23];
	dig_P9 = dig_P9<<8 | trimdata[22];
	dig_H1 = trimdata[24];
	//dig_H2 = (trimdata[26]<<8) |trimdata[25] ;
	dig_H2 = trimdata[26];
	dig_H2 = dig_H2<<8 | trimdata[25];
	dig_H3 = trimdata[27];
	//dig_H4 = (trimdata[28]<<4) | (trimdata[29]&0x0f);
	dig_H4 = trimdata[28];
	dig_H4 = dig_H4<<4 | (trimdata[29]&0x0f);

	dig_H5 = (trimdata[30]<<4) | (trimdata[29]>>4);
	dig_H5 = trimdata[30];
	dig_H5=dig_H5<<4 | (trimdata[29]&0x0f);
	dig_H6 = trimdata[31];

}

int BMP_Config(uint8_t osrs_t, uint8_t osrs_h, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
TrimRead();
uint8_t datatowrite;

//reset the device
datatowrite = 0xB6;
HAL_I2C_Mem_Write(&hi2c1,0xEC,CTRL_HUM_REG,1,&datatowrite,1,1000);
HAL_Delay(100);
//oversampling temperature and pressure
datatowrite = 0x27;
HAL_I2C_Mem_Write(&hi2c1,0xEC, CTRL_MEAS_REG,1,&datatowrite,1,1000);
//oversampling humidity
datatowrite = 0x01;
HAL_I2C_Mem_Write(&hi2c1,0xEC,CTRL_HUM_REG,1,&datatowrite,1,1000);
//mode initialization

//standby initialization

//filter initialization
datatowrite = (t_sb <<5) |(filter << 2);
HAL_I2C_Mem_Write(&hi2c1,0xEC,CONFIG_REG,1,&datatowrite,1,1000);

return 0;
}
void BMP_WakeUP(void){

}
void BMP_Measure(void){
	TrimRead();
	HAL_Delay(1000);
	BME_Read_Raw();
	Temperature = (BME280_compensate_T_int32 (tRaw))/100.0;
	Pressure = (BME280_compensate_P_int64 (pRaw))/256.0;
	Humidity = (BME280_compensate_H_int32 (hRaw))/1024.0;
}

void BME_Read_Raw(void){
	 uint8_t rawData[8];
	 HAL_I2C_Mem_Read(&hi2c1,0xEC,PRESS_REG_MSB,1,rawData,8,HAL_MAX_DELAY);
	 pRaw = (rawData[0]<<12)|(rawData[1]<<4) | (rawData[2]>>4);
	 tRaw = (rawData[3]<<12) |(rawData[4]<<4 | rawData[5]>>4);
	 hRaw = (rawData[6]<<8) | rawData[7];
}

uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}


int32_t BME280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))>> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t BME280_compensate_H_int32(int32_t adc_H)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) *\
			v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *\
					((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +\
							((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) +\
					8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *\
			((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}
