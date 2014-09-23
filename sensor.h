#ifndef SENSOR_H
#define SENSOR_H

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define PI       3.14159
#define deltaT   0.05

///////////////////////////////   variables   ///////////////////////////////
// I2C
int fd_mpu6050 = 0, fd_ms5611 = 0, fd_hmc5883 = 0;

// MPU6050 , magnetometer     raw data
struct xyz_axis {
	uint8_t X_h, X_l, Y_h, Y_l, Z_h, Z_l;
	float X, Y, Z;
	float XValue, YValue, ZValue;
} accel_xyz, gyro_xyz, mag_xyz;

struct accelValue{
	float p,r,y;
	float pitch,roll,yaw;
}ACCEL;

struct gyroValue{
	float Xd,Yd,Zd;
	float pitch,roll,yaw;
}GYRO;

//after filter, (almost)real status
struct quadCopter{
	float pitch,roll,yaw,altitude;
}QUADCOPTER;

// magnetometer value
double magZ = 0.0;//degree

//barometer
double ms5611_C[6], ms5611_D[2];
float tempc, press_now, tempR, pr;
unsigned int temp_h,temp_l;
float baroH;

//initial sensor
void sensorInit();// I2C address setup

//get sensor value
void getGyroValue();
void getMagValue();
void getBaroValue();
void getAccelValue();

// calculate: pitch,roll,yaw
void accelCalculate();
void gyroCalculate();

//filter
void sensorFilter(float ,float ,float ,float);
float filterFunction(float,float,float); //float ratio , float highPassValue , float lowPassValue
//absolute value
float myAbs(float value);

///////////////////////////////   function   ///////////////////////////////

void sensorInit() {	
	printf("initializing....\n");

	unsigned int ms5611_promaddr = 0xA2;	
	int i = 0;

	////////// I2C setup
	//mpu6050	
	fd_mpu6050 = wiringPiI2CSetup (0x68);
	wiringPiI2CWriteReg8 (fd_mpu6050,0x6A,0x00);  //i2c master enable
	wiringPiI2CWriteReg8 (fd_mpu6050,0x37,0x02);  //i2c bypass enable
	wiringPiI2CWriteReg8 (fd_mpu6050,0x6B,0x01); //set the sleep unenable,chose clock source.
	//mpu6050 sampling rate setup
	wiringPiI2CWriteReg8 (fd_mpu6050,0x19,0x00); //set sample rate divide = 0
	wiringPiI2CWriteReg8 (fd_mpu6050,0x1A,0x00); //set gyroscope output rate = 8khz

	//ms5611
	fd_ms5611 = wiringPiI2CSetup (0x77);
	wiringPiI2CWriteReg8 (fd_ms5611,0xEE, 0x1E);

	//hmc5883
	fd_hmc5883 = wiringPiI2CSetup(0x1E);
	wiringPiI2CWriteReg8 (fd_hmc5883,0x02,0x00);

	delayMicroseconds(300);

	wiringPiI2CWriteReg8(fd_hmc5883,0x00,0x70);// TODO:(0x78)75 Hz,01111000, 0x70(default)

	delay(1000);

	////////// ms5611 getting coefficients 
	while( i < 6 ) {
		temp_h = wiringPiI2CReadReg8(fd_ms5611, ms5611_promaddr+2*i);
		temp_l = wiringPiI2CReadReg8(fd_ms5611, ms5611_promaddr+1+2*i);
		ms5611_C[i++] = (temp_h<<8)+temp_l;

		//printf("ms5611_C[%x]: %f\n", ms5611_promaddr+2*(i-1), ms5611_C[i-1]);
	}

	////////// initial quadcopter bias & get first data
	//barometer...height	
	float aveH = 0.0;
	i = 0 ; 
	while( i < 30 ){
		getBaroValue();
		aveH = aveH + baroH;
		i++;
	}
	aveH = aveH / 30.0;

	//accel...pitch & roll 
	getAccelValue();
	ACCEL.p = sqrt( accel_xyz.XValue * accel_xyz.XValue + accel_xyz.ZValue * accel_xyz.ZValue );
	ACCEL.r = sqrt( accel_xyz.YValue * accel_xyz.YValue + accel_xyz.ZValue * accel_xyz.ZValue );
	ACCEL.y = sqrt( accel_xyz.XValue * accel_xyz.XValue + accel_xyz.YValue * accel_xyz.YValue );

	ACCEL.pitch = atan( accel_xyz.YValue / ACCEL.p ) * 180 / PI;
	ACCEL.roll  = atan( accel_xyz.XValue / ACCEL.r ) * 180 / PI;

	if(myAbs(ACCEL.pitch)<4)
		ACCEL.pitch = 0;
	if(myAbs(ACCEL.roll)<2)
		ACCEL.roll = 0;

	//magnetometer
	getMagValue();
	
	//set bias of quadcopter
	GYRO.pitch = ACCEL.pitch;
	GYRO.roll = ACCEL.roll;
	GYRO.yaw = magZ;

	QUADCOPTER.pitch = ACCEL.pitch;
	QUADCOPTER.roll  = ACCEL.roll;
	QUADCOPTER.yaw  = magZ;
	QUADCOPTER.altitude = aveH;

}

void getGyroValue() {
	
	gyro_xyz.X_h = wiringPiI2CReadReg8(fd_mpu6050, 0x43);
	gyro_xyz.X_l = wiringPiI2CReadReg8(fd_mpu6050, 0x44);

	gyro_xyz.Y_h = wiringPiI2CReadReg8(fd_mpu6050, 0x45);
	gyro_xyz.Y_l = wiringPiI2CReadReg8(fd_mpu6050, 0x46);

	gyro_xyz.Z_h = wiringPiI2CReadReg8(fd_mpu6050, 0x47);
	gyro_xyz.Z_l = wiringPiI2CReadReg8(fd_mpu6050, 0x48);


	gyro_xyz.X = (gyro_xyz.X_h<<8) + gyro_xyz.X_l;
	gyro_xyz.Y = (gyro_xyz.Y_h<<8) + gyro_xyz.Y_l;
	gyro_xyz.Z = (gyro_xyz.Z_h<<8) + gyro_xyz.Z_l;

	gyro_xyz.XValue = (float) gyro_xyz.X/131;
	gyro_xyz.YValue = (float) gyro_xyz.Y/131;
	gyro_xyz.ZValue = (float) gyro_xyz.Z/131;	
}
void gyroCalculate(){   // gyro xyz  -> pitch,roll,yaw
	// gyro X offset
	if ( myAbs( gyro_xyz.XValue ) > 4 )
		GYRO.Xd = gyro_xyz.XValue;
	else
		GYRO.Xd = 0.0;

	// gyro Y offset
	if ( myAbs( gyro_xyz.YValue ) > 2 )
		GYRO.Yd = gyro_xyz.YValue - 1.212214;
	else
		GYRO.Yd = 0.0;

	// gyro Z offset
	if ( myAbs( gyro_xyz.ZValue ) > 2 )
		GYRO.Zd = gyro_xyz.ZValue - 0.912214;
	else
		GYRO.Zd=0.0;
	//pitch 
	GYRO.pitch = GYRO.pitch + GYRO.Xd * deltaT;
	//roll
	GYRO.roll  = GYRO.roll  + GYRO.Yd * deltaT;
	//yaw
	GYRO.yaw   = GYRO.yaw   + GYRO.Zd * deltaT;

}
void getMagValue() {
	
	//while(wiringPiI2CReadReg8(fd_hmc5883,0x09)&&0x01==1) { }//sensor not ready

	mag_xyz.X_h = wiringPiI2CReadReg8(fd_hmc5883, 0x3);
	mag_xyz.X_l = wiringPiI2CReadReg8(fd_hmc5883, 0x4);

	mag_xyz.Y_h = wiringPiI2CReadReg8(fd_hmc5883, 0x5);
	mag_xyz.Y_l = wiringPiI2CReadReg8(fd_hmc5883, 0x6);

	mag_xyz.Z_h = wiringPiI2CReadReg8(fd_hmc5883, 0x7);
	mag_xyz.Z_l = wiringPiI2CReadReg8(fd_hmc5883, 0x8);

	mag_xyz.X = (mag_xyz.X_h<<8) + mag_xyz.X_l + 78;
	mag_xyz.Y = (mag_xyz.Y_h<<8) + mag_xyz.Y_l - 108;
	mag_xyz.Z = (mag_xyz.Z_h<<8) + mag_xyz.Z_l;

	magZ = atan2(mag_xyz.Y, mag_xyz.X);
	//if(angleZ < 0) angleZ += 2 * PI;
	
	magZ = magZ * 360.0 / 2 / PI; // -180~180
}

void getBaroValue() {
	unsigned int testR_h, testR_l;
	unsigned int testTemp16H, testTemp16L;
	double dt, temp, sens, p, off, p0;

	wiringPiI2CWriteReg16 (fd_ms5611, 0x48, 0x00);
	delay(20);
	testR_h = wiringPiI2CReadReg16(fd_ms5611, 0x00);
	testR_l = wiringPiI2CReadReg16(fd_ms5611, 0x01);

	unsigned int tempp = testR_h>>8;
	unsigned int testR = ((testR_h-(tempp<<8))<<16) + (tempp<<8) + (testR_l>>8);
	ms5611_D[0] = testR;

	wiringPiI2CWriteReg16 (fd_ms5611, 0x58, 0x00);
	delay(20);
	testTemp16H = wiringPiI2CReadReg16(fd_ms5611, 0x00);
	testTemp16L = wiringPiI2CReadReg16(fd_ms5611, 0x01);
	unsigned int tempT = testTemp16H>>8;
	unsigned int testResult = ((testTemp16H - (tempT<<8))<<16) + (tempT<<8) + (testTemp16L>>8);
	ms5611_D[1] = testResult;

	dt = ms5611_D[1] - ms5611_C[4]*(1<<8); 
	temp = 2000 + dt*ms5611_C[5]/(1<<23);
	off = ms5611_C[1]*(1<<16) + ms5611_C[3]*dt/(1<<7);
	sens = ms5611_C[0]*(1<<15) + ms5611_C[2]*dt/(1<<8);
	p = (ms5611_D[0]*(sens/(1<<21))-off)/(1<<15);
	tempc = temp/100;
	tempR = tempc+237.15;
	press_now = p/100;
	p0 = 1013.25;
	pr = pow((p0/press_now),1/5.257)-1;
	baroH = pr*tempR/0.0065;

}

void getAccelValue() {
	accel_xyz.X_h = wiringPiI2CReadReg8(fd_mpu6050, 0x3b);
	accel_xyz.X_l = wiringPiI2CReadReg8(fd_mpu6050, 0x3c);

	accel_xyz.Y_h = wiringPiI2CReadReg8(fd_mpu6050, 0x3d);
	accel_xyz.Y_l = wiringPiI2CReadReg8(fd_mpu6050, 0x3e);

	accel_xyz.Z_h = wiringPiI2CReadReg8(fd_mpu6050, 0x3f);
	accel_xyz.Z_l = wiringPiI2CReadReg8(fd_mpu6050, 0x40);


	accel_xyz.X = (accel_xyz.X_h<<8) + accel_xyz.X_l;
	accel_xyz.Y = (accel_xyz.Y_h<<8) + accel_xyz.Y_l;
	accel_xyz.Z = (accel_xyz.Z_h<<8) + accel_xyz.Z_l;

	accel_xyz.XValue = (float) accel_xyz.X/16384;
	accel_xyz.YValue = (float) accel_xyz.Y/16384;
	accel_xyz.ZValue = (float) accel_xyz.Z/16384;

}

void accelCalculate(){     //   Xg,Yg,Zg -> degree of pitch roll yaw
	ACCEL.p = sqrt( accel_xyz.XValue * accel_xyz.XValue + accel_xyz.ZValue * accel_xyz.ZValue );
	ACCEL.r = sqrt( accel_xyz.YValue * accel_xyz.YValue + accel_xyz.ZValue * accel_xyz.ZValue );
	ACCEL.y = sqrt( accel_xyz.XValue * accel_xyz.XValue + accel_xyz.YValue * accel_xyz.YValue );

	ACCEL.pitch = atan( accel_xyz.YValue / ACCEL.p ) * 180 / PI;
	ACCEL.roll  = -atan( accel_xyz.XValue / ACCEL.r ) * 180 / PI;
	ACCEL.yaw   = atan( ACCEL.y / accel_xyz.ZValue ) * 180 / PI;
}

void sensorFilter(float pitchRatio,float rollRatio,float yawRatio,float altitudeRatio){
	float move = myAbs(ACCEL.pitch) + myAbs(ACCEL.roll) + myAbs(ACCEL.yaw);
	//filterFunction( float ratio , float highPassValue , float lowPassValue  ) 
	if(move > 5){
		QUADCOPTER.pitch = filterFunction(pitchRatio, QUADCOPTER.pitch + gyro_xyz.XValue * deltaT ,ACCEL.pitch);
		QUADCOPTER.roll  = filterFunction(rollRatio,  QUADCOPTER.roll  + gyro_xyz.YValue * deltaT ,ACCEL.roll );
		
		//yaw is still a problem
		//QUADCOPTER.yaw   = filterFunction(yawRatio,   QUADCOPTER.yaw   + gyro_xyz.ZValue * deltaT ,ACCEL.yaw  );
		//ratio = 0.94
		//QUADCOPTER.yaw = filterFunction(rollRatio,  QUADCOPTER.yaw ,magZ ) ; // with LPF (only compass information) ,photo 1, -180 to 180 gg 
		QUADCOPTER.yaw = magZ; // without filter ,photo 2 ,-180 to 180 nearly right (sampling rate ??)
		//QUADCOPTER.yaw = filterFunction(rollRatio,  QUADCOPTER.yaw + gyro_xyz.ZValue * deltaT , magZ ); // have -180 to 180 problem

	}

	//baro...height
	QUADCOPTER.altitude = filterFunction( altitudeRatio , QUADCOPTER.altitude , baroH );

}

float myAbs(float value){
   if(value>=0)
      return value;
   else
      return -value;
}

float filterFunction( float ratio , float highPassValue , float lowPassValue  ){
	float filterValue = 0.0;	
	filterValue = ratio * highPassValue + (1-ratio) * lowPassValue;
	return filterValue;
}


#endif
