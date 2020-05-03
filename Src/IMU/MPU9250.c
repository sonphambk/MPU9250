/*
 * MPU9250.c
 *
 *  Created on: 30 thg 4, 2020
 *      Author: PC
 */
#include "i2c.h"
#include "MPU9250.h"
#include "MPU9250_register.h"
#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "stdio.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim3;




char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}

void init_IMU()
{

	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;

	uint8_t d[2];

	/* Check if device is connected */
	while(HAL_I2C_IsDeviceReady(&hi2c1,mpu_address,2,3) != HAL_OK);

	/* Wakeup MPU6050 */
	/* Try to transmit via I2C */
	d[0] = PWR_MGMT_1;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

	Check_Connection();
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	mpu9250_set_clock_source(MPU9250_CLOCK_PLL_XGYRO);

	//CONGFIG
	d[0] = CONFIG;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

	/* Set data sample rate */
	d[0] = SMPLRT_DIV;   // sample rate = SAMPLE_RATE/(1 + 7) = 1khz
	d[1] = SAMPLE_RATE_1khz;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);
	// config accelerometer
	Set_Accel_Range(MPU9250_ACCEL_FS_2);
	// config gyro
	Set_Gyro_Range(MPU9250_GYRO_FS_2000);
	d[0] = USER_CTRL;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

}
void init_magnetometer()
{
	uint8_t mag_address = MAG_ADDRESS_DEFAULT;
	uint8_t d[2];

	while(HAL_I2C_IsDeviceReady(&hi2c1,mag_address,3,200)!=HAL_OK);
	d[0] = CNTL;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,(uint8_t *)d,2,1000) != HAL_OK);
	HAL_Delay(10000);
	/*MODE[3:0]: Operation mode setting
	"0000": Power-down mode
	"0001": Single measurement mode
	"0010": Continuous measurement mode 1
	"0110": Continuous measurement mode 2
	"0100": External trigger measurement mode
	"1000": Self-test mode
	"1111": Fuse ROM access mode
	Other code settings are prohibited
	BIT: Output bit setting
	"0": 14-bit output
	"1": 16-bit output */

	d[0] = CNTL;
	d[1] = 0x1F;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,(uint8_t *)d,2,1000) != HAL_OK);
	uint8_t raw_data[3];
	uint8_t Sensitivity = ASAX ;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,&Sensitivity,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,mag_address,(uint8_t*)raw_data,3,1000) != HAL_OK);


	asax = (float)(raw_data[0] - 128)/256.0f +1.0f;  // Return x-axis sensitivity adjustment values, etc.
	asay = (float)(raw_data[1] - 128)/256.0f +1.0f;
	asaz = (float)(raw_data[2] - 128)/256.0f +1.0f;

	d[0] = CNTL;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,(uint8_t *)d,2,1000) != HAL_OK);

	MPU9250_MAG_FULL_SCALE mag_fs = MFS_16BITS;
	d[0] = CNTL;
	d[1] = (mag_fs<<4)|0x02;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,(uint8_t *)d,2,1000) != HAL_OK);
	/*
	 *
	 * REG : CNTL
	 *  *MODE[3:0]: Operation mode setting
	"0000": Power-down mode
	"0001": Single measurement mode
	"0010": Continuous measurement mode 1
	"0110": Continuous measurement mode 2
	"0100": External trigger measurement mode
	"1000": Self-test mode
	"1111": Fuse ROM access mode
	Other code settings are prohibited
	BIT: Output bit setting
	"0": 14-bit output
	"1": 16-bit output*/
}


void Process_IMU()
{
	//read raw data
	uint8_t data[14];
	uint8_t reg = ACCEL_XOUT_H;
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;


	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address, data, 14, 1000) != HAL_OK);

	/*-------- Accel ---------*/
	Accel_x = (uint16_t)(( data[0] << 8 ) | data[1]);
	Accel_y = (uint16_t)(( data[2] << 8 ) | data[3]);
	Accel_z = (uint16_t)(( data[4] << 8 ) | data[5]);

	/*-------- Gyrometer --------*/
	Gyro_x = (uint16_t)(( data[8] << 8  ) | data[9]);
	Gyro_y = (uint16_t)(( data[10] << 8 ) | data[11]);
	Gyro_z = (uint16_t)(( data[12] << 8 ) | data[13]);


	Accel_x = (Accel_x - Accel_x_bias)/accel_sensitivity;
	Accel_y = (Accel_y - Accel_y_bias)/accel_sensitivity;
	Accel_z = (Accel_z - Accel_z_bias)/accel_sensitivity ;

	Gyro_x = (Gyro_x - Gyro_x_bias)/gyro_sensitivity;
	Gyro_y = (Gyro_y - Gyro_y_bias)/gyro_sensitivity;
	Gyro_z = (Gyro_z - Gyro_z_bias)/gyro_sensitivity;



}
void Set_Accel_Range(MPU9250_ACCEL_FULL_SCALE accel_FS)
{
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];

	d[0] = ACCEL_CONFIG;
	d[1] = accel_FS;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

	d[0] = ACCEL_CONFIG2;
	d[1] = 0x0A;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,1000)!=HAL_OK);
}
void Calibration_IMU()
{
	uint8_t data[14];
	uint8_t reg = ACCEL_XOUT_H;

	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;




	for(int i=0; i < 100; i++){
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg,1,1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address, data, 14, 1000) != HAL_OK);

				/*-------- Accel ---------*/
		Accel_x = (uint16_t)(( data[0] << 8 ) | data[1]);
		Accel_y = (uint16_t)(( data[2] << 8 ) | data[3]);
		Accel_z = (uint16_t)(( data[4] << 8 ) | data[5]);

				/*-------- Gyrometer --------*/
		Gyro_x = (uint16_t)(( data[8]  << 8 ) | data[9]);
		Gyro_y = (uint16_t)(( data[10] << 8 ) | data[11]);
		Gyro_z = (uint16_t)(( data[12] << 8 ) | data[13]);

		Accel_x_bias += Accel_x;
		Accel_y_bias += Accel_y;
		Accel_z_bias += Accel_z;

		Gyro_x_bias += Gyro_x;
		Gyro_y_bias += Gyro_y;
		Gyro_z_bias += Gyro_z;
	}
	Accel_x_bias /= 100;
	Accel_y_bias /= 100;
	Accel_z_bias /= 100;

	Gyro_x_bias /= 100;
	Gyro_y_bias /= 100;
	Gyro_z_bias /= 100;

	if(Accel_z_bias > 0) //// Remove gravity from the z-axis accelerometer bias calculation
	{
		Accel_z_bias -= (int16_t)accel_sensitivity;
	}
	else
	{
		Accel_z_bias += (int16_t)accel_sensitivity;
	}
}

void Set_Gyro_Range(MPU9250_GYRO_FULL_SCALE gyro_FS)
{

	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];
	d[0] = GYRO_CONFIG;
	d[1] = gyro_FS;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

}
bool Check_Connection()
{
	// Check WHO_AM_I
	uint8_t who_i_am = (uint8_t)WHO_AM_I_MPU9250;
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;


	uint8_t temp;
    while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&who_i_am,1,1000)!= HAL_OK);
    while(HAL_I2C_Master_Receive(&hi2c1,mpu_address,&temp,1,1000) != HAL_OK);
    return (temp == 0x71);

}
void mpu9250_set_clock_source(MPU9250_clock_source_t clock_source)
{
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];

	d[0] = PWR_MGMT_1;
	d[1] = clock_source;
	while(HAL_I2C_Master_Transmit(&hi2c1,mpu_address,(uint8_t *)d,2,100)!= HAL_OK);
}

void Reset_MPU()
{
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];
	d[0] = PWR_MGMT_1;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,mpu_address,(uint8_t *)d,2,100)!= HAL_OK);
	HAL_Delay(1000);
}





