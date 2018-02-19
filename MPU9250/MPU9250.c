/*
 * MPU9250.c
 *
 *  Created on: 2017/09/14
 *      Author: feunoir
 */

#include "MPU9250.h"
#include "MPU9250_RegisterMap.h"

#include "util/inv_mpu.h"

static unsigned char mpu9250_orientation;
static unsigned char tap_count;
static unsigned char tap_direction;
static bool _tap_available;
static void orient_cb(unsigned char orient);
static void tap_cb(unsigned char direction, unsigned char count);


void MPU9250_GetHandle(MPU9250_t* mpu9250, I2C_HandleTypeDef* hi2cx)
{
	mpu9250->hi2c = hi2cx;
}

void MPU9250_SetAddress(MPU9250_t* mpu9250, MPU9250_Address_t Address)
{

	mpu9250->address = Address;
}

inv_error_t MPU9250_begin(MPU9250_t * mpu9250)
{
	inv_error_t result;
    struct int_param_s int_param;

	//Wire.begin();

	result = mpu_init(&int_param, mpu9250->hi2c, mpu9250->address);

	if (result)
		return result;

	mpu_set_bypass(1); // Place all slaves (including compass) on primary bus

	MPU9250_setSensors(mpu9250, INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

	mpu9250->_mSense = 6.665f;
	mpu9250->_gSense = MPU9250_getGyroSens(mpu9250);
	mpu9250->_aSense = MPU9250_getAccelSens(mpu9250);

	return result;
}

inv_error_t MPU9250_enableInterrupt(MPU9250_t * mpu9250, unsigned char enable)
{
	return set_int_enable(enable);
}

inv_error_t MPU9250_setIntLevel(MPU9250_t * mpu9250, unsigned char active_low)
{
	return mpu_set_int_level(active_low);
}

inv_error_t MPU9250_setIntLatched(MPU9250_t * mpu9250, unsigned char enable)
{
	return mpu_set_int_latched(enable);
}

short MPU9250_getIntStatus(MPU9250_t * mpu9250)
{
	short status;
	if (mpu_get_int_status(&status) == INV_SUCCESS)
	{
		return status;
	}
	return 0;
}

// Accelerometer Low-Power Mode. Rate options:
// 1.25 (1), 2.5 (2), 5, 10, 20, 40,
// 80, 160, 320, or 640 Hz
// Disables compass and gyro
inv_error_t MPU9250_lowPowerAccel(MPU9250_t * mpu9250, unsigned short rate)
{
	return mpu_lp_accel_mode(rate);
}

inv_error_t MPU9250_setGyroFSR(MPU9250_t * mpu9250, unsigned short fsr)
{
	inv_error_t err;
	err = mpu_set_gyro_fsr(fsr);
	if (err == INV_SUCCESS)
	{
		mpu9250->_gSense = MPU9250_getGyroSens(mpu9250);
	}
	return err;
}

inv_error_t MPU9250_setAccelFSR(MPU9250_t * mpu9250, unsigned char fsr)
{
	inv_error_t err;
	err = mpu_set_accel_fsr(fsr);
	if (err == INV_SUCCESS)
	{
		mpu9250->_aSense = MPU9250_getAccelSens(mpu9250);
	}
	return err;
}

unsigned short MPU9250_getGyroFSR(MPU9250_t * mpu9250)
{
	unsigned short tmp;
	if (mpu_get_gyro_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

unsigned char MPU9250_getAccelFSR(MPU9250_t * mpu9250)
{
	unsigned char tmp;
	if (mpu_get_accel_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

unsigned short MPU9250_getMagFSR(MPU9250_t * mpu9250)
{
	unsigned short tmp;
	if (mpu_get_compass_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t MPU9250_setLPF(MPU9250_t * mpu9250, unsigned short lpf)
{
	return mpu_set_lpf(lpf);
}

unsigned short MPU9250_getLPF(MPU9250_t * mpu9250)
{
	unsigned short tmp;
	if (mpu_get_lpf(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t MPU9250_setSampleRate(MPU9250_t * mpu9250, unsigned short rate)
{
    return mpu_set_sample_rate(rate);
}

unsigned short MPU9250_getSampleRate(MPU9250_t * mpu9250)
{
	unsigned short tmp;
	if (mpu_get_sample_rate(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t MPU9250_setCompassSampleRate(MPU9250_t * mpu9250, unsigned short rate)
{
	return mpu_set_compass_sample_rate(rate);
}

unsigned short MPU9250_getCompassSampleRate(MPU9250_t * mpu9250)
{
	unsigned short tmp;
	if (mpu_get_compass_sample_rate(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}

	return 0;
}

float MPU9250_getGyroSens(MPU9250_t * mpu9250)
{
	float sens;
	if (mpu_get_gyro_sens(&sens) == INV_SUCCESS)
	{
		return sens;
	}
	return 0;
}

unsigned short MPU9250_getAccelSens(MPU9250_t * mpu9250)
{
	unsigned short sens;
	if (mpu_get_accel_sens(&sens) == INV_SUCCESS)
	{
		return sens;
	}
	return 0;
}

float MPU9250_getMagSens(MPU9250_t * mpu9250)
{
	return 0.15; // Static, 4915/32760
}

unsigned char MPU9250_getFifoConfig(MPU9250_t * mpu9250)
{
	unsigned char sensors;
	if (mpu_get_fifo_config(&sensors) == INV_SUCCESS)
	{
		return sensors;
	}
	return 0;
}

inv_error_t MPU9250_configureFifo(MPU9250_t * mpu9250, unsigned char sensors)
{
	return mpu_configure_fifo(sensors);
}

inv_error_t MPU9250_resetFifo(MPU9250_t * mpu9250)
{
	return mpu_reset_fifo();
}

unsigned short MPU9250_fifoAvailable(MPU9250_t * mpu9250)
{
	unsigned char fifoH, fifoL;

	if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
		return 0;
	if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
		return 0;

	return (fifoH << 8 ) | fifoL;
}

inv_error_t MPU9250_updateFifo(MPU9250_t * mpu9250)
{
	short gyro[3], accel[3];
	unsigned long timestamp;
	unsigned char sensors, more;

	if (mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more) != INV_SUCCESS)
		return INV_ERROR;

	if (sensors & INV_XYZ_ACCEL)
	{
		mpu9250->ax = accel[X_AXIS];
		mpu9250->ay = accel[Y_AXIS];
		mpu9250->az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		mpu9250->gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		mpu9250->gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		mpu9250->gz = gyro[Z_AXIS];

	mpu9250->time = timestamp;

	return INV_SUCCESS;
}

inv_error_t MPU9250_setSensors(MPU9250_t * mpu9250, unsigned char sensors)
{
	return mpu_set_sensors(sensors);
}

bool MPU9250_dataReady(MPU9250_t * mpu9250)
{
	unsigned char intStatusReg;

	if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS)
	{
		return (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
	}
	return false;
}

inv_error_t MPU9250_update(MPU9250_t * mpu9250, unsigned char sensors)
{
	inv_error_t aErr = INV_SUCCESS;
	inv_error_t gErr = INV_SUCCESS;
	inv_error_t mErr = INV_SUCCESS;
	inv_error_t tErr = INV_SUCCESS;

	if (sensors & UPDATE_ACCEL)
		aErr = MPU9250_updateAccel(mpu9250);
	if (sensors & UPDATE_GYRO)
		gErr = MPU9250_updateGyro(mpu9250);
	if (sensors & UPDATE_COMPASS)
		mErr = MPU9250_updateCompass(mpu9250);
	if (sensors & UPDATE_TEMP)
		tErr = MPU9250_updateTemperature(mpu9250);

	return aErr | gErr | mErr | tErr;
}

int MPU9250_updateAccel(MPU9250_t * mpu9250)
{
	short data[3];

	if (mpu_get_accel_reg(data, (unsigned long *)mpu9250->time))
	{
		return INV_ERROR;
	}
	mpu9250->ax = data[X_AXIS];
	mpu9250->ay = data[Y_AXIS];
	mpu9250->az = data[Z_AXIS];
	return INV_SUCCESS;
}

int MPU9250_updateGyro(MPU9250_t * mpu9250)
{
	short data[3];

	if (mpu_get_gyro_reg(data, (unsigned long *)mpu9250->time))
	{
		return INV_ERROR;
	}
	mpu9250->gx = data[X_AXIS];
	mpu9250->gy = data[Y_AXIS];
	mpu9250->gz = data[Z_AXIS];
	return INV_SUCCESS;
}

int MPU9250_updateCompass(MPU9250_t * mpu9250)
{
	short data[3];

	if (mpu_get_compass_reg(data, (unsigned long *)mpu9250->time))
	{
		return INV_ERROR;
	}
	mpu9250->mx = data[X_AXIS];
	mpu9250->my = data[Y_AXIS];
	mpu9250->mz = data[Z_AXIS];
	return INV_SUCCESS;
}

inv_error_t MPU9250_updateTemperature(MPU9250_t * mpu9250)
{
	return mpu_get_temperature((long *)mpu9250->temperature, (unsigned long *)mpu9250->time);
}

int MPU9250_selfTest(MPU9250_t * mpu9250, unsigned char debug)
{
	long gyro[3], accel[3];
	return mpu_run_self_test(gyro, accel);
}

inv_error_t MPU9250_dmpBegin(MPU9250_t * mpu9250, unsigned short features, unsigned short fifoRate)
{
	unsigned short feat = features;
	unsigned short rate = fifoRate;

	if (MPU9250_dmpLoad(mpu9250) != INV_SUCCESS)
		return INV_ERROR;

	// 3-axis and 6-axis LP quat are mutually exclusive.
	// If both are selected, default to 3-axis
	if (feat & DMP_FEATURE_LP_QUAT)
	{
		feat &= ~(DMP_FEATURE_6X_LP_QUAT);
		dmp_enable_lp_quat(1);
	}
	else if (feat & DMP_FEATURE_6X_LP_QUAT)
		dmp_enable_6x_lp_quat(1);

	if (feat & DMP_FEATURE_GYRO_CAL)
		dmp_enable_gyro_cal(1);

	if (MPU9250_dmpEnableFeatures(mpu9250, feat) != INV_SUCCESS)
		return INV_ERROR;

	rate = constrain(rate, 1, 200);
	if (MPU9250_dmpSetFifoRate(mpu9250, rate) != INV_SUCCESS)
		return INV_ERROR;

	return mpu_set_dmp_state(1);
}

inv_error_t MPU9250_dmpLoad(MPU9250_t * mpu9250)
{
	return dmp_load_motion_driver_firmware();
}

unsigned short MPU9250_dmpGetFifoRate(MPU9250_t * mpu9250)
{
	unsigned short rate;
	if (dmp_get_fifo_rate(&rate) == INV_SUCCESS)
		return rate;

	return 0;
}

inv_error_t MPU9250_dmpSetFifoRate(MPU9250_t * mpu9250, unsigned short rate)
{
	if (rate > MAX_DMP_SAMPLE_RATE) rate = MAX_DMP_SAMPLE_RATE;
	return dmp_set_fifo_rate(rate);
}

inv_error_t MPU9250_dmpUpdateFifo(MPU9250_t * mpu9250)
{
	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors;
	unsigned char more;

	if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)
		   != INV_SUCCESS)
    {
	   return INV_ERROR;
    }

	if (sensors & INV_XYZ_ACCEL)
	{
		mpu9250->ax = accel[X_AXIS];
		mpu9250->ay = accel[Y_AXIS];
		mpu9250->az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		mpu9250->gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		mpu9250->gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		mpu9250->gz = gyro[Z_AXIS];
	if (sensors & INV_WXYZ_QUAT)
	{
		mpu9250->qw = quat[0];
		mpu9250->qx = quat[1];
		mpu9250->qy = quat[2];
		mpu9250->qz = quat[3];
	}

	mpu9250->time = timestamp;

	return INV_SUCCESS;
}

inv_error_t MPU9250_dmpEnableFeatures(MPU9250_t * mpu9250, unsigned short mask)
{
	unsigned short enMask = 0;
	enMask |= mask;
	// Combat known issue where fifo sample rate is incorrect
	// unless tap is enabled in the DMP.
	enMask |= DMP_FEATURE_TAP;
	return dmp_enable_feature(enMask);
}

unsigned short MPU9250_dmpGetEnabledFeatures(MPU9250_t * mpu9250)
{
	unsigned short mask;
	if (dmp_get_enabled_features(&mask) == INV_SUCCESS)
		return mask;
	return 0;
}

inv_error_t MPU9250_dmpSetTap(MPU9250_t * mpu9250,
        unsigned short xThresh, unsigned short yThresh, unsigned short zThresh,
        unsigned char taps, unsigned short tapTime, unsigned short tapMulti)
{
	unsigned char axes = 0;
	if (xThresh > 0)
	{
		axes |= TAP_X;
		xThresh = constrain(xThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<X_AXIS, xThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (yThresh > 0)
	{
		axes |= TAP_Y;
		yThresh = constrain(yThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<Y_AXIS, yThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (zThresh > 0)
	{
		axes |= TAP_Z;
		zThresh = constrain(zThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<Z_AXIS, zThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (dmp_set_tap_axes(axes) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_count(taps) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_time(tapTime) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_time_multi(tapMulti) != INV_SUCCESS)
		return INV_ERROR;

    dmp_register_tap_cb(tap_cb);

	return INV_SUCCESS;
}

unsigned char MPU9250_getTapDir(MPU9250_t * mpu9250)
{
	_tap_available = false;
	return tap_direction;
}

unsigned char MPU9250_getTapCount(MPU9250_t * mpu9250)
{
	_tap_available = false;
	return tap_count;
}

bool MPU9250_tapAvailable(MPU9250_t * mpu9250)
{
	return _tap_available;
}

inv_error_t MPU9250_dmpSetOrientation(MPU9250_t * mpu9250, const signed char * orientationMatrix)
{
	unsigned short scalar;
	scalar = orientation_row_2_scale(orientationMatrix);
	scalar |= orientation_row_2_scale(orientationMatrix + 3) << 3;
	scalar |= orientation_row_2_scale(orientationMatrix + 6) << 6;

    dmp_register_android_orient_cb(orient_cb);

	return dmp_set_orientation(scalar);
}

unsigned char MPU9250_dmpGetOrientation(MPU9250_t * mpu9250)
{
	return mpu9250_orientation;
}

inv_error_t MPU9250_dmpEnable3Quat(MPU9250_t * mpu9250)
{
	unsigned short dmpFeatures;

	// 3-axis and 6-axis quat are mutually exclusive
	dmpFeatures = MPU9250_dmpGetEnabledFeatures(mpu9250);
	dmpFeatures &= ~(DMP_FEATURE_6X_LP_QUAT);
	dmpFeatures |= DMP_FEATURE_LP_QUAT;

	if (MPU9250_dmpEnableFeatures(mpu9250, dmpFeatures) != INV_SUCCESS)
		return INV_ERROR;

	return dmp_enable_lp_quat(1);
}

unsigned long MPU9250_dmpGetPedometerSteps(MPU9250_t * mpu9250)
{
	unsigned long steps;
	if (dmp_get_pedometer_step_count(&steps) == INV_SUCCESS)
	{
		return steps;
	}
	return 0;
}

inv_error_t MPU9250_dmpSetPedometerSteps(MPU9250_t * mpu9250, unsigned long steps)
{
	return dmp_set_pedometer_step_count(steps);
}

unsigned long MPU9250_dmpGetPedometerTime(MPU9250_t * mpu9250)
{
	unsigned long walkTime;
	if (dmp_get_pedometer_walk_time(&walkTime) == INV_SUCCESS)
	{
		return walkTime;
	}
	return 0;
}

inv_error_t MPU9250_dmpSetPedometerTime(MPU9250_t * mpu9250, unsigned long time)
{
	return dmp_set_pedometer_walk_time(time);
}

float MPU9250_calcAccel(MPU9250_t * mpu9250, int axis)
{
	return (float) axis / (float) mpu9250->_aSense;
}

float MPU9250_calcGyro(MPU9250_t * mpu9250, int axis)
{
	return (float) axis / (float) mpu9250->_gSense;
}

float MPU9250_calcMag(MPU9250_t * mpu9250, int axis)
{
	return (float) axis / (float) mpu9250->_mSense;
}

float MPU9250_calcQuat(MPU9250_t * mpu9250, long axis)
{
	return qToFloat(axis, 30);
}

float qToFloat(long number, unsigned char q)
{
	unsigned long mask = 0;
	for (int i=0; i<q; i++)
	{
		mask |= (1<<i);
	}
	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

void MPU9250_computeEulerAngles(MPU9250_t * mpu9250, bool degrees)
{
    float dqw = qToFloat(mpu9250->qw, 30);
    float dqx = qToFloat(mpu9250->qx, 30);
    float dqy = qToFloat(mpu9250->qy, 30);
    float dqz = qToFloat(mpu9250->qz, 30);

    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;

	// Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;

    mpu9250->pitch = asin(t2) * 2;
    mpu9250->roll = atan2(t3, t4);
    mpu9250->yaw = atan2(t1, t0);

	if (degrees)
	{
		mpu9250->pitch *= (180.0 / M_PI);
		mpu9250->roll *= (180.0 / M_PI);
		mpu9250->yaw *= (180.0 / M_PI);
		if (mpu9250->pitch < 0) mpu9250->pitch = 360.0 + mpu9250->pitch;
		if (mpu9250->roll < 0) mpu9250->roll = 360.0 + mpu9250->roll;
		if (mpu9250->yaw < 0) mpu9250->yaw = 360.0 + mpu9250->yaw;
	}
}

float MPU9250_computeCompassHeading(MPU9250_t * mpu9250)
{
	if (mpu9250->my == 0)
		mpu9250->heading = (mpu9250->mx < 0) ? 180.0 : 0;
	else
		mpu9250->heading = atan2(mpu9250->mx, mpu9250->my);

	if (mpu9250->heading > M_PI) mpu9250->heading -= (2 * M_PI);
	else if (mpu9250->heading < -M_PI) mpu9250->heading += (2 * M_PI);
	else if (mpu9250->heading < 0) mpu9250->heading += 2 * M_PI;

	mpu9250->heading*= 180.0 / M_PI;

	return mpu9250->heading;
}

unsigned short orientation_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}

static void tap_cb(unsigned char direction, unsigned char count)
{
	_tap_available = true;
	tap_count = count;
	tap_direction = direction;
}

static void orient_cb(unsigned char orient)
{
	mpu9250_orientation = orient;
}
