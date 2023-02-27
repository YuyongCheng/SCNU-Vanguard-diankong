#ifndef _IMU_H_
#define _IMU_H_


#define BMI088_Chip_ID_Add 0X00
#define BMI088_Gyro_X_LSB_Add 0x02
#define BMI088_Gyro_X_MSB_Add 0x03
#define BMI088_Gyro_Y_LSB_Add 0x04
#define BMI088_Gyro_Y_MSB_Add 0x05
#define BMI088_Gyro_Z_LSB_Add 0x06
#define BMI088_Gyro_Z_MSB_Add 0x07
#define BMI088_Accel_X_LSB_Add 0X12
#define BMI088_Accel_X_MSB_Add 0X13
#define BMI088_Accel_Y_LSB_Add 0X14
#define BMI088_Accel_Y_MSB_Add 0X15
#define BMI088_Accel_Z_LSB_Add 0X16
#define BMI088_Accel_Z_MSB_Add 0X17
#define BMI088_Temp_MSB_Add 0x22    //低三位  [2:0]
#define BMI088_Temp_LSB_Add 0x23    //高8位   [10:3]
#define GYRO_RANGE_ADDR 0x0F
#define GYRO_RANGE_2000_DEG_S 0x00
#define GYRO_RANGE_1000_DEG_S 0x01
#define GYRO_RANGE_500_DEG_S 0x02
#define GYRO_RANGE_250_DEG_S 0x03
#define GYRO_RANGE_125_DEG_S 0x04

#define GYRO_BANDWIDTH_ADDR 0x10
#define GYRO_ODR_2000Hz_BANDWIDTH_532Hz 0x00
#define GYRO_ODR_2000Hz_BANDWIDTH_230Hz 0x01
#define GYRO_ODR_1000Hz_BANDWIDTH_116Hz 0x02
#define GYRO_ODR_400Hz_BANDWIDTH_47Hz 0x03
#define GYRO_ODR_200Hz_BANDWIDTH_23Hz 0x04
#define GYRO_ODR_100Hz_BANDWIDTH_12Hz 0x05
#define GYRO_ODR_200Hz_BANDWIDTH_64Hz 0x06
#define GYRO_ODR_100Hz_BANDWIDTH_32Hz 0x07

typedef struct
{
	uint8_t x_MSB_Data;
	uint8_t x_LSB_Data;
	uint8_t y_MSB_Data;
	uint8_t y_LSB_Data;
	uint8_t z_MSB_Data;
	uint8_t z_LSB_Data;
	uint8_t temp;
	int sensitivity;

}IMU_TypeDef;
typedef enum
{
	accel = 0,
	gyro = 1
}accel_or_gyro;
extern IMU_TypeDef BMI088_Accel, BMI088_Gyro;
extern void BMI088_read_Gyro(IMU_TypeDef *imu);
extern uint8_t BMI088_read_Accel(uint8_t const reg);
extern void BMI088_init();

#endif
