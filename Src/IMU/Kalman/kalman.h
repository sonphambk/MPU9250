/*
 * kalman.h
 *
 *  Created on: 30 thg 9, 2020
 *      Author: PC
 */

#ifndef IMU_KALMAN
#define IMU_KALMAN

static float Q_angle;
static float Q_bias;
static float R_measure;
static float P[2][2];
static float f_bias;
static float f_angle;
static float rate;
float y ,new_yaw;

float roll_kal,pitch_kal,yaw_kal;
void kalman_init(void);
float get_kalman_angle(float new_angle,float new_gyro,float dt);




#endif /* IMU_KALMAN */
