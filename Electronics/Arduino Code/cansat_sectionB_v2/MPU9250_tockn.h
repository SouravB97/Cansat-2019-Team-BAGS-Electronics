#ifndef MPU9250_TOCKN_H
#define MPU9250_TOCKN_H

#include "Arduino.h"
#include "Wire.h"

#define MPU9250_ADDR         0x68
#define MPU9250_SMPLRT_DIV   0x19
#define MPU9250_CONFIG       0x1a
#define MPU9250_GYRO_CONFIG  0x1b
#define MPU9250_ACCEL_CONFIG 0x1c
#define MPU9250_WHO_AM_I     0x75
#define MPU9250_PWR_MGMT_1   0x6b
#define MPU9250_TEMP_H       0x41
#define MPU9250_TEMP_L       0x42


class MPU9250{
	public:

	MPU9250(TwoWire &w);
	MPU9250(TwoWire &w, float aC, float gC);

	void begin();

	void setGyroOffsets(float x, float y, float z);

	void writeMPU9250(byte reg, byte data);
	byte readMPU9250(byte reg);

	int16_t getRawAccX(){ return rawAccX; };
	int16_t getRawAccY(){ return rawAccY; };
	int16_t getRawAccZ(){ return rawAccZ; };

	int16_t getRawTemp(){ return rawTemp; };

	int16_t getRawGyroX(){ return rawGyroX; };
	int16_t getRawGyroY(){ return rawGyroY; };
	int16_t getRawGyroZ(){ return rawGyroZ; };

	float getTemp(){ return temp; };

	float getAccX(){ return accX; };
	float getAccY(){ return accY; };
	float getAccZ(){ return accZ; };

	float getGyroX(){ return gyroX; };
	float getGyroY(){ return gyroY; };
	float getGyroZ(){ return gyroZ; };
	
	int calibrateMag();
    float getMagBiasX_uT();
    float getMagScaleFactorX();
    float getMagBiasY_uT();
    float getMagScaleFactorY();
    float getMagBiasZ_uT();
    float getMagScaleFactorZ();
	void setMagCalX(float bias,float scaleFactor);
    void setMagCalY(float bias,float scaleFactor);
    void setMagCalZ(float bias,float scaleFactor);
	
	float getMagX(){ return magX; };
	float getMagY(){ return magY; };
	float getMagZ(){ return magZ; };

	void calcGyroOffsets(bool console = false);

	float getGyroXoffset(){ return gyroXoffset; };
	float getGyroYoffset(){ return gyroYoffset; };
	float getGyroZoffset(){ return gyroZoffset; };

	void update();

	float getAccAngleX(){ return angleAccX; };
	float getAccAngleY(){ return angleAccY; };

	float getGyroAngleX(){ return angleGyroX; };
	float getGyroAngleY(){ return angleGyroY; };
	float getGyroAngleZ(){ return angleGyroZ; };

	float getAngleX(){ return angleX; };
	float getAngleY(){ return angleY; };
	float getAngleZ(){ return angleZ; };

	//private:

	TwoWire *wire;

	int16_t rawAccX, rawAccY, rawAccZ, rawTemp,
	rawGyroX, rawGyroY, rawGyroZ, rawMagX, rawMagY, rawMagZ;

	float gyroXoffset, gyroYoffset, gyroZoffset;

	float temp, accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ;

	float angleGyroX, angleGyroY, angleGyroZ,
	angleAccX, angleAccY, angleAccZ;

	float angleX, angleY, angleZ;

  float interval;
	long preInterval;

	float accCoef, gyroCoef;
};

#endif
