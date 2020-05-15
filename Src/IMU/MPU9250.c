/*
 * MPU9250.c
 *
 *  Created on: 30 thg 4, 2020
 *      Author: PC
 */
#include "i2c.h"
#include "MPU9250.h"
#include "MPU9250_register.h"
#include "Madgwick/Madgwick.h"
#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim3;


void Calib_magnetometer();

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
	HAL_Delay(100);

	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	mpu9250_set_clock_source(MPU9250_CLOCK_PLL_XGYRO);

	//CONGFIG
	d[0] = CONFIG;
	d[1] = 0x00;//0x05
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

	/* Set data sample rate */
	d[0] = SMPLRT_DIV;   // sample rate = SAMPLE_RATE/(1 + 7) = 1khz
	d[1] = SAMPLE_RATE_1khz;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);
	// config accelerometer
	Set_Accel_Range(MPU9250_ACCEL_FS_2);
	// config gyro
	Set_Gyro_Range(MPU9250_GYRO_FS_250);
	d[0] = USER_CTRL;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);
	d[0] = INT_ENABLE;
	d[1] = 0x01;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,1000)!=HAL_OK);
	d[0] = INT_PIN_CFG;
	d[1] = 0x22;  // turn on AK8963
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,1000)!=HAL_OK);

}
void init_magnetometer()
{
	uint8_t mag_address = MAG_ADDRESS_DEFAULT;
	uint8_t d[2];

	while(HAL_I2C_IsDeviceReady(&hi2c1,mag_address,3,200)!=HAL_OK);
	d[0] = CNTL;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,(uint8_t *)d,2,1000) != HAL_OK);
	HAL_Delay(100);
	d[0] = CNTL;
	d[1] = 0x1F;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,(uint8_t *)d,2,1000) != HAL_OK);
	HAL_Delay(100);
	uint8_t raw_data[3];
	uint8_t Sensitivity = ASAX ;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,&Sensitivity,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,mag_address,(uint8_t*)raw_data,3,1000) != HAL_OK);

	asax = (float)(raw_data[0] - 128)/256.0f +1.0f;  // Return x-axis sensitivity adjustment values, etc.
	asay = (float)(raw_data[1] - 128)/256.0f +1.0f;
	asaz = (float)(raw_data[2] - 128)/256.0f +1.0f;
	//Please note that we has to change the chip to power-down mode first then switch it to another mode
	 //reset the Magnetometer to power down mode

	d[0] = CNTL;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,(uint8_t *)d,2,1000) != HAL_OK);
	HAL_Delay(100);
	MPU9250_MAG_FULL_SCALE mag_fs = MFS_16BITS;
	d[0] = CNTL;
	d[1] = (mag_fs<<4)|0x06;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,(uint8_t *)d,2,1000) != HAL_OK);
	HAL_Delay(100);    //wait for the mode changes
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
	Accel_x = (int16_t)((int16_t)( data[0] << 8 ) | data[1]);
	Accel_y = (int16_t)((int16_t)( data[2] << 8 ) | data[3]);
	Accel_z = (int16_t)((int16_t)( data[4] << 8 ) | data[5]);

	/*-------- Gyrometer --------*/
	Gyro_x = (int16_t)((int16_t)( data[8] << 8  ) | data[9]);
	Gyro_y = (int16_t)((int16_t)( data[10] << 8 ) | data[11]);
	Gyro_z = (int16_t)((int16_t)( data[12] << 8 ) | data[13]);


	Accel_X = (float)((int32_t)Accel_x - Accel_x_bias)/(float)accel_sensitivity;
	Accel_Y =  (float)((int32_t)Accel_y - Accel_y_bias)/(float)accel_sensitivity;
	Accel_Z =  (float)((int32_t)Accel_z - Accel_z_bias)/(float)accel_sensitivity ;

	Gyro_X =  (float)((int32_t)Gyro_x - Gyro_x_bias)/(float)gyro_sensitivity;
	Gyro_Y =  (float)((int32_t)Gyro_y - Gyro_y_bias)/(float)gyro_sensitivity;
	Gyro_Z =  (float)((int32_t)Gyro_z - Gyro_z_bias)/(float)gyro_sensitivity;

	// Get data of Magnetometer
	Get_magnetometer();

	//MadgwickAHRSupdateIMU(Gyro_X*M_PI/180.0f,Gyro_Y*M_PI/180.0f,Gyro_Z*M_PI/180.0f,Accel_X,Accel_Y,Accel_Z);
	MadgwickAHRSupdate(Gyro_X*M_PI/180.0f,Gyro_Y*M_PI/180.0f,Gyro_Z*M_PI/180.0f,Accel_X,Accel_Y,Accel_Z,Mag_X,Mag_Y,Mag_Z);
	//MahonyAHRSupdate(Gyro_X*M_PI/180.0f,Gyro_Y*M_PI/180.0f,Gyro_Z*M_PI/180.0f,Accel_X,Accel_Y,Accel_Z,Mag_X,Mag_Y,Mag_Z);

}
void Set_Accel_Range(MPU9250_ACCEL_FULL_SCALE accel_FS)
{
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];
	uint8_t reg1 = ACCEL_CONFIG;
	//uint8_t reg2 = ACCEL_CONFIG2;

	uint8_t c;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg1,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address,&c, 1, 1000) != HAL_OK);
//	c[0] &=  ~0x18; // Clear AFS bits [4:3]
//	c[0] |= accel_FS; // Set full scale range for the gyro

	d[0] = ACCEL_CONFIG;
	d[1] = (c & 0xE7) | (uint8_t)accel_FS ;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);


//	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg2,1,1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address,c, 1, 1000) != HAL_OK);
//	c[0] &= ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
//	c[0] |=  0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	d[0] = ACCEL_CONFIG2;
	d[1] = 0x05;
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
		Accel_x = (int16_t)( (int16_t)( data[0] << 8 ) | data[1]);
		Accel_y = (int16_t)( (int16_t)( data[2] << 8 ) | data[3]);
		Accel_z = (int16_t)( (int16_t)( data[4] << 8 ) | data[5]);

				/*-------- Gyrometer --------*/
		Gyro_x = (int16_t)( (int16_t)( data[8]  << 8 ) | data[9]);
		Gyro_y = (int16_t)( (int16_t)( data[10] << 8 ) | data[11]);
		Gyro_z = (int16_t)( (int16_t)( data[12] << 8 ) | data[13]);

		Accel_x_bias += (int32_t)Accel_x;
		Accel_y_bias += (int32_t)Accel_y;
		Accel_z_bias += (int32_t)Accel_z;

		Gyro_x_bias += (int32_t)Gyro_x;
		Gyro_y_bias += (int32_t)Gyro_y;
		Gyro_z_bias += (int32_t)Gyro_z;
	}
	Accel_x_bias /= 100;
	Accel_y_bias /= 100;
	Accel_z_bias /= 100;

	Gyro_x_bias /= 100;
	Gyro_y_bias /= 100;
	Gyro_z_bias /= 100;

	if(Accel_z_bias > 0) //// Remove gravity from the z-axis accelerometer bias calculation
	{
		Accel_z_bias -= (int32_t)accel_sensitivity;
	}
	else
	{
		Accel_z_bias += (int32_t)accel_sensitivity;
	}
}

void Set_Gyro_Range(MPU9250_GYRO_FULL_SCALE gyro_FS)
{

//	uint8_t reg = GYRO_CONFIG;
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];
//	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg,1,1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address,c, 1, 1000) != HAL_OK);
//	c[0] &=  ~0x02; // Clear Fchoice bits [1:0]
//	c[0] &=  ~0x18; // Clear AFS bits [4:3]
//	c[0] |= gyro_FS;
	d[0] = GYRO_CONFIG;
	d[1] = gyro_FS;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

}
int Check_Connection(uint8_t return_true_val)
{
	// Check WHO_AM_I
	uint8_t who_i_am = (uint8_t)WHO_AM_I_MPU9250;
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;


    while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&who_i_am,1,1000)!= HAL_OK);
    while(HAL_I2C_Master_Receive(&hi2c1,mpu_address,&temp,1,1000) != HAL_OK);
    if (temp == return_true_val)
    	return 1;
    else
    	return 0;

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

void Complementary_filter(float Gyro_x,float Gyro_y,float Gyro_z,float  Acc_x,float Acc_y,float Acc_z, float dt)
{
	float accel_roll = atan2((double)Acc_y,(double)Acc_z)*RAD2DEC;
	float accel_pitch = atan2((double)-Acc_x,(double)sqrt(Acc_y*Acc_y + Acc_z*Acc_z))*RAD2DEC;
	roll = alpha*(roll + dt*Gyro_x) + (1-alpha)*accel_roll;
	pitch = alpha*(pitch + dt*Gyro_y) + (1-alpha)*accel_pitch;
}
void Get_magnetometer()
{
	uint8_t raw_data[7];
	uint8_t reg_ST1 = ST1;
	uint8_t mag_address = MAG_ADDRESS_DEFAULT;
	uint8_t reg = XOUT_L;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,&reg_ST1,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)mag_address,raw_data,1,1000) != HAL_OK);
	if (raw_data[0] & 0x01)
	{
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,&reg,1,1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)mag_address,raw_data,7,1000) != HAL_OK);
		// Read the six raw data and ST2 registers sequentially into data arra
		if(!(raw_data[6] & 0x08))// Check if magnetic sensor overflow set, if not then report data
		{
			Mag_x = (int16_t)((raw_data[1]<<8) | raw_data[0] );
			Mag_y = (int16_t)((raw_data[3]<<8) | raw_data[2] );
			Mag_z = (int16_t)((raw_data[5]<<8) | raw_data[4] );
		}

//		uint8_t magbias[3];
//		 magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//		 magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
//		 magbias[2] = +125.;
		//convert uT --> mG
//		Mag_x = Mag_x*asax*mag_sensitivity*scale_x;
//		Mag_y = Mag_y*asay*mag_sensitivity*scale_y;
//		Mag_z = Mag_z*asaz*mag_sensitivity*scale_z;
//		Mag_x = (Mag_x-mag_bias[0])*asax*scale_mag;
//		Mag_y = (Mag_y-mag_bias[1])*asay*scale_mag;
//		Mag_z = (Mag_z-mag_bias[2])*asaz*scale_mag;
		Mag_X = ((float)Mag_x * asax * mag_sensitivity - mag_offset[0])*scale_x;
		Mag_Y = ((float)Mag_y * asay * mag_sensitivity - mag_offset[1])*scale_y;
		Mag_Z = ((float)Mag_z * asaz * mag_sensitivity - mag_offset[2])*scale_z;
	}
}
void Calib_magnetometer()
{
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};
	uint8_t raw_data[7];
	mag_offset[3] = 0;
	uint8_t reg_ST1 = ST1;
	uint8_t mag_address = MAG_ADDRESS_DEFAULT;
	uint8_t reg = XOUT_L;
	uint16_t i = 0;
	for (i = 0; i < 1500;i++)
	{
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,&reg_ST1,1,1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)mag_address,raw_data,1,1000) != HAL_OK);
		if (raw_data[0] & 0x01)
		{
			while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mag_address,&reg,1,1000) != HAL_OK);
			while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)mag_address,raw_data,7,1000) != HAL_OK);
			// Read the six raw data and ST2 registers sequentially into data arra
			if(!(raw_data[6] & 0x08))// Check if magnetic sensor overflow set, if not then report data
			{
				Mag_x = (int16_t)((int16_t)(raw_data[1]<<8) | raw_data[0] );
				Mag_y = (int16_t)((int16_t)(raw_data[3]<<8) | raw_data[2] );
				Mag_z = (int16_t)((int16_t)(raw_data[5]<<8) | raw_data[4] );
			}
		}
	int16_t mag_temp[3] = {Mag_x,Mag_y,Mag_z};
	for (int j = 0;j < 3;j++)
	{
		if (mag_temp[j] > mag_max[j])  mag_max[j] = mag_temp[j];
		if (mag_temp[j] < mag_min[j])  mag_min[j] = mag_temp[j];
	}
		HAL_Delay(12);
	}
	// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0])/2;
	mag_bias[1] = (mag_max[1] + mag_min[1])/2;
	mag_bias[2] = (mag_max[2] + mag_min[2])/2;

	mag_offset[0] = (float)(mag_bias[0]) * mag_sensitivity * asax;
	mag_offset[1] = (float)(mag_bias[1]) * mag_sensitivity * asay;
	mag_offset[2] = (float)(mag_bias[2]) * mag_sensitivity * asaz;
	// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0])/2;
	mag_scale[1] = (mag_max[1] - mag_min[1])/2;
	mag_scale[2] = (mag_max[2] - mag_min[2])/2;

	scale_x = 0,scale_y = 0,scale_z = 0;
	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;


	scale_x = avg_rad/(float)mag_scale[0];    //1.14
	scale_y = avg_rad/(float)mag_scale[1];  // 1.00
	scale_z = avg_rad/(float)mag_scale[2]; // 0.89



}
void Quaternion_to_EulerAngle(float w,float x,float y,float z)
{
	  // roll (x-axis rotation)
		float sinr = 2*(w*x + y*z);
		float cosr = 1 - 2*(x*x + y*y);
		roll = atan2(sinr,cosr);

	 // pitch (y-axis rotation)
		float sinp = 2*(w*y - z*x);
		if (abs(sinp) >= 1)
			pitch = copysign(M_PI/2,sinp);
		else
			pitch =  asin(sinp);
	//// yaw (z-axis rotation)
		float siny = 2 *(w*z + x*y);
		float cosy = 1 - 2*(y*y + z*z);
		yaw = atan2(siny,cosy);

		roll = roll*180/M_PI;
		pitch = pitch*180/M_PI;
		yaw = yaw*180/M_PI;
}

