#ifndef bmp280
#define bmp280
#include "stm32f4xx.h"
//
typedef struct  {
	uint16_t first[2];
	int16_t rest[10];
}calibration_data1;
typedef struct {
	float temp;
	float pressure;
	float bpm_altetude;
}truedata;
extern calibration_data1 calibration_data;
extern truedata true_data;
//
void bpm_init_calibration(void);
void bpm_read_data(void);
#endif
