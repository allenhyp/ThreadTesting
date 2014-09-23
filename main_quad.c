#include "sensor.h"
#include "plot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <stdint.h>

void printResult();
void control_event(int sig);
int showError();


int dataNum = 0;// indicate the data number
float pitchRatio = 0.98, rollRatio = 0.98, yawRatio = 0.98, altitudeRatio = 0.93;

int main(){

	//wiringPi check
	showError();
	// ctl + c ...exiting
	(void)signal(SIGINT,control_event);
	(void)signal(SIGQUIT,control_event);

	//gnuplot file
	gnuPlotOpenfile();

	//sensor initial
	sensorInit();

	printf("....start loop....\n");
	while(1) {
		//accel
		getAccelValue();
		accelCalculate();
		//gyro
		getGyroValue();	
		gyroCalculate();	
		//mag		
		getMagValue();
		//baro		
		getBaroValue();

		//filter
		sensorFilter( pitchRatio, rollRatio, yawRatio, altitudeRatio );

		//put data to file
		gnuPlotPutToFile(dataNum);	

		printResult();
		delay(10);//+40(in getTempValue)....sampling time
		dataNum++;
	}
}


/*****************  printResult  ************************/
void printResult() {

	printf("===============================================================\n");
	printf("# %d\n", dataNum);
	//raw data
	printf("Acceleration(X: %.3f, Y: %.3f, Z: %.3f)\n", accel_xyz.XValue, accel_xyz.YValue, accel_xyz.ZValue);
	printf("Gyroscope   (X: %.3f, Y: %.3f, Z: %.3f)\n", gyro_xyz.XValue, gyro_xyz.YValue, gyro_xyz.ZValue);
	printf("Magnetometer(X: %d, Y: %d, Z: %d), AngleZ  (%.3f)\n", mag_xyz.X, mag_xyz.Y, mag_xyz.Z, magZ);
	printf("Temperature (%.3f), Pressure (%.3f) , Barometer (%.3f) \n", tempc, press_now, baroH);
	printf("\n");
	printf("ACCEL pitch: %.3f  , roll: %.3f  , yaw: %.3f    \n",ACCEL.pitch,ACCEL.roll,ACCEL.yaw);
	printf("GYRO  pitch: %.3f  , roll: %.3f  , yaw: %.3f    \n",GYRO.pitch,GYRO.roll,GYRO.yaw);
	printf("QUAD  pitch: %.3f  , roll: %.3f  , yaw: %.3f    \n",QUADCOPTER.pitch,QUADCOPTER.roll,QUADCOPTER.yaw);
	printf("QUAD  altitude   (%.3f) \n",QUADCOPTER.altitude);
}
/*****************  showError  ************************/
int showError(){
	//show error
	if( getuid() != 0){//wiringPi requires root privilege
		printf("Error:wiringPi must be run as root.\n");
       		return 1;
	}
   	if(wiringPiSetup()==-1){
       		printf("Error:wiringPi setup failed.\n");
       		return 1;
   	}
	return 0;
}
/*****************  control event  ************************/
void control_event(int sig){

	printf("\n---------   EXITING   ---------\n");

	gnuPlot();

	exit(0);

}
