/*
 * Madgwick.h
 *
 *  Created on: 5 thg 5, 2020
 *      Author: PC
 */

#ifndef IMU_MADGWICK_MADGWICK_H_
#define IMU_MADGWICK_MADGWICK_H_

// Variable declaration

//extern volatile float beta;				// algorithm gain

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
float q[4];

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
           // vector to hold integral error for Mahony method
//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
#endif /* IMU_MADGWICK_MADGWICK_H_ */
