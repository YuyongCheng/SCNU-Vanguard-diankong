#include "main.h"
#include "spi.h"
#include "IMU.h"
#include "usart.h"
IMU_TypeDef BMI088_Accel, BMI088_Gyro;
uint8_t spi_TxData, spi_RxData;
uint8_t BMI088_read_Accel(uint8_t const reg)
 {
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, RESET);
	spi_TxData = reg | 0x80;		//使地�?第一位为1（读模式�?
	HAL_SPI_TransmitReceive(&hspi1, &spi_TxData, &spi_RxData, 1, 55);	//写入�?要读取的地址
	HAL_SPI_TransmitReceive(&hspi1, &spi_TxData, &spi_RxData, 1, 55);	//写入�?要读取的地址
	HAL_SPI_TransmitReceive(&hspi1, &spi_TxData, &spi_RxData, 1, 55);  //接受读取信息
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, SET);    //传输停止
	return spi_RxData;
 }
void BMI088_read_Gyro(IMU_TypeDef *imu)
 {
	uint8_t temp_arr[6];
	HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, RESET);
	spi_TxData = 0x02 | 0x80;		                       //使地�?第一位为1（读模式�?
	HAL_SPI_Transmit(&hspi1, &spi_TxData, 1, 300);	       //写入�?要读取的地址
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

	for(int i=0; i<6; i++)                                 //接受读取信息
	{
		HAL_SPI_Receive(&hspi1, &temp_arr[i], 1, 300);
		while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	}

	imu->x_LSB_Data = temp_arr[0];
	imu->x_MSB_Data = temp_arr[1];
	imu->y_LSB_Data = temp_arr[2];
	imu->y_MSB_Data = temp_arr[3];
	imu->z_LSB_Data = temp_arr[4];
	imu->z_MSB_Data = temp_arr[5];
	HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, SET);    //传输停止

 }
void BMI088_write_byte(uint8_t write_data, uint8_t addr, accel_or_gyro a_enum)
{
	switch(a_enum)
	{
	case accel:
		HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, RESET);
		break;
    case gyro:
		HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, RESET);
		break;
	}
	spi_TxData = addr & 0x7F;                    //bit0为0，向imu写
	HAL_SPI_Transmit(&hspi1, &spi_TxData, 1, 500);
	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_SPI_Transmit(&hspi1, &write_data, 1, 500);
	HAL_Delay(30);
	switch(a_enum)
	{
	case accel:
	    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, SET);
		break;
	case gyro:
		HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, SET);
		break;
	}
}

void BMI088_init()
{
	//加速度计初始化
    BMI088_write_byte(0xB6, 0x7E, accel);            //向0x7E写入0xb6以软件复位加速度计
	BMI088_write_byte(0x04, 0x7D, accel);            //向0x7D写入0x04以取消加速度计暂停


	//陀螺仪初始化
	BMI088_write_byte(0xB6, 0x14, gyro);             //向0x14写入0xb6以软件复位陀螺仪
	BMI088_write_byte(0x00,  0x11, gyro);
	BMI088_write_byte(GYRO_RANGE_500_DEG_S, GYRO_RANGE_ADDR, gyro);
	BMI088_write_byte(GYRO_ODR_200Hz_BANDWIDTH_64Hz, GYRO_BANDWIDTH_ADDR, gyro);
	BMI088_Accel.sensitivity = 1;
	BMI088_Gyro.sensitivity = 1;


}
