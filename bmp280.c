#include "bmp280.h"
#include <math.h>
#include "stm32f4xx.h"
//
#define addres (0x76<<1)
#define calibration_add 0x88
#define REG_CONFIG 0xF5
#define REG_CTRL_MEAS 0xF4
#define data_add 0xF7
//
extern I2C_HandleTypeDef hi2c1;
static int32_t t_fine;

calibration_data1 calibration_data;
truedata true_data;
//
	int32_t compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    // T1 is first[0], T2 is rest[0], T3 is rest[1]
    var1 = ((((adc_T >> 3) - ((int32_t)calibration_data.first[0] << 1))) * ((int32_t)calibration_data.rest[0])) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calibration_data.first[0])) * ((adc_T >> 4) - ((int32_t)calibration_data.first[0]))) >> 12) * ((int32_t)calibration_data.rest[1])) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;}

    uint32_t compensate_P(int32_t adc_P) {
        int64_t var1, var2, p;
        // P1 is first[1]
        // P2..P9 are rest[2]..rest[9]
        var1 = ((int64_t)t_fine) - 128000;
        var2 = var1 * var1 * (int64_t)calibration_data.rest[6]; // P6
        var2 = var2 + ((var1 * (int64_t)calibration_data.rest[5]) << 17); // P5
        var2 = var2 + (((int64_t)calibration_data.rest[4]) << 35); // P4
        var1 = ((var1 * var1 * (int64_t)calibration_data.rest[3]) >> 8) + ((var1 * (int64_t)calibration_data.rest[2]) << 12); // P3 & P2
        var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration_data.first[1]) >> 33; // P1

        if (var1 == 0) return 0; // Avoid division by zero

        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)calibration_data.rest[9]) * (p >> 13) * (p >> 13)) >> 25; // P9
        var2 = (((int64_t)calibration_data.rest[8]) * p) >> 19; // P8
        p = ((p + var1 + var2) >> 8) + (((int64_t)calibration_data.rest[7]) << 4); // P7
        return (uint32_t)p;
    }
//
void bpm_init_calibration(void){
	uint8_t data[24];
	HAL_I2C_Mem_Read(&hi2c1, addres, calibration_add, 1, data, 24, 100);
	calibration_data.first[0]=(data[1]<<8 | data[0]);
	calibration_data.rest[0]=(data[3]<<8 | data[2]);
	calibration_data.rest[1]=(data[5]<<8 | data[4]);
	calibration_data.first[1]=(data[7]<<8 | data[6]);
	calibration_data.rest[2]=(data[9]<<8 | data[8]);
	calibration_data.rest[3]=(data[11]<<8 | data[10]);
	calibration_data.rest[4]=(data[13]<<8 | data[12]);
	calibration_data.rest[5]=(data[15]<<8 | data[14]);
	calibration_data.rest[6]=(data[17]<<8 | data[16]);
	calibration_data.rest[7]=(data[19]<<8 | data[18]);
	calibration_data.rest[8]=(data[21]<<8 | data[20]);
	calibration_data.rest[9]=(data[23]<<8 | data[22]);

	uint8_t config = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, addres, REG_CONFIG, 1, &config, 1, 100);

	uint8_t measure = 0x57;
	HAL_I2C_Mem_Write(&hi2c1, addres, REG_CTRL_MEAS, 1, &measure, 1, 100);
}

void bpm_read_data(void){
	uint8_t real_data[6];
	HAL_I2C_Mem_Read(&hi2c1, addres, data_add, 1, real_data, 6, 5);
	int32_t presure_data =real_data[0]<<12 | real_data[1]<<4 | real_data[2]>>4;
	int32_t tem =real_data[3]<<12 | real_data[4]<<4 | real_data[5]>>4;
	int32_t tem_data = compensate_T(tem);
	true_data.temp=tem_data/100.0f;
	uint32_t press_int = compensate_P(presure_data);
	true_data.pressure = press_int / 256.0f;
	true_data.bpm_altetude = 44330.0f * (1.0f - powf(true_data.pressure / 101325.0f, 0.1903f));

}


