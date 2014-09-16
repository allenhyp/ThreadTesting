#include<wiringPiI2C.h>
#include <wiringPi.h>
#include<stdio.h>
#include<string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include "int.h"

#define dt 0.006              // 500ms sample rate
#define pi 3.14159265359
#define a 0.98                  // complementary filter use 

//gnuplot
#define NUM_POINTS 9999
FILE * dataFILE;
FILE * gnuplotPipe ;
FILE * gnuplotPipe1 ;

#define testPin 2

void control_event(int sig);





/************************************************************/
int main(){

// ctl + c ...exiting
(void)signal(SIGINT,control_event);
(void)signal(SIGQUIT,control_event);


int fd;

fd = wiringPiI2CSetup (0x68);
wiringPiI2CWriteReg8 (fd,0x6B,0x01); //set the sleep unenable,chose clock source.
wiringPiI2CWriteReg8 (fd,0x19,0x00); //set sample rate divide = 0
wiringPiI2CWriteReg8 (fd,0x1A,0x00); //set gyroscope output rate = 8khz

printf("set 0x6B=%Xn",wiringPiI2CReadReg8 (fd,0x6B));


unsigned char Xgh,Xgl,Ygh,Ygl,Zgh,Zgl, Xh,Xl,Yh,Yl,Zh,Zl;
short Xvalue, Yvalue, Zvalue, Xgyro, Ygyro, Zgyro;
float Xg, Yg, Zg, Xdegree, Ydegree, Zdegree, pa, ra, ya, Xd, Yd, Zd, pitchACC;
float  roll, yaw, rollACC, yawACC,  roll_cf, yaw_cf;

/*
Xgh = wiringPiI2CReadReg8(fd, 0x3B);
Xgl = wiringPiI2CReadReg8(fd, 0x3C);

Ygh = wiringPiI2CReadReg8(fd, 0x3D);
Ygl = wiringPiI2CReadReg8(fd, 0x3E);

Zgh = wiringPiI2CReadReg8(fd, 0x3F);
Zgl = wiringPiI2CReadReg8(fd, 0x40);

Xvalue = Xgh * 256 + Xgl;
Yvalue = Ygh * 256 + Ygl;
Zvalue = Zgh * 256 + Zgl;

Xg = (float)Xvalue / 16384;
Yg = (float)Yvalue / 16384;
Zg = (float)Zvalue / 16384;

pa = sqrt(Xg*Xg + Zg*Zg);
ra = sqrt(Yg*Yg + Zg*Zg);
ya = sqrt(Xg*Xg + Yg*Yg);

pitchACC = atan(Yg/pa)*180/pi;
*/

float pitch = function(), pitch_cf = function();
float fma;

//gpio
wiringPiSetup();
pinMode(testPin,OUTPUT);
digitalWrite(testPin,LOW);

//file
int i = 0;
dataFILE = fopen("data.temp", "w");
gnuplotPipe = popen ("gnuplot -persistent", "w");
gnuplotPipe1 = popen ("gnuplot -persistent", "w");


while(1){

Xgh = wiringPiI2CReadReg8(fd, 0x3B);
Xgl = wiringPiI2CReadReg8(fd, 0x3C);

Ygh = wiringPiI2CReadReg8(fd, 0x3D);
Ygl = wiringPiI2CReadReg8(fd, 0x3E);

Zgh = wiringPiI2CReadReg8(fd, 0x3F);
Zgl = wiringPiI2CReadReg8(fd, 0x40);

Xvalue = Xgh * 256 + Xgl;
Yvalue = Ygh * 256 + Ygl;
Zvalue = Zgh * 256 + Zgl;

Xg = (float)Xvalue / 16384;
Yg = (float)Yvalue / 16384;
Zg = (float)Zvalue / 16384;

pa = sqrt(Xg*Xg + Zg*Zg);
ra = sqrt(Yg*Yg + Zg*Zg);
ya = sqrt(Xg*Xg + Yg*Yg);

pitchACC = atan(Yg/pa)*180/pi;
rollACC = atan(Xg/ra)*180/pi;
yawACC = atan(ya/Zg)*180/pi;

//printf(" \n Xgh: %d, Xgl: %d\n", Xh, Xgl);
//printf(" Ygh: %d, Ygl: %d\n", Ygh, Ygl);
//printf(" Zgh: %d, Zgl: %d\n\n", Zgh, Zgl);
//printf(" Xvalue: %d, Yvalue: %d, Zvalue: %d\n",Xvalue,Yvalue,Zvalue);


// gyro

Xh = wiringPiI2CReadReg8(fd, 0x43);
Xl = wiringPiI2CReadReg8(fd, 0x44);
Xgyro = Xh * 256 + Xl;

Yh = wiringPiI2CReadReg8(fd, 0x45);
Yl = wiringPiI2CReadReg8(fd, 0x46);
Ygyro = Yh * 256 + Yl;

Zh = wiringPiI2CReadReg8(fd, 0x47);
Zl = wiringPiI2CReadReg8(fd, 0x48);
Zgyro = Zh * 256 + Zl;

Xdegree = (float)Xgyro / 131;
Ydegree = -(float)Ygyro / 131;
Zdegree = (float)Zgyro / 131;

// gyro X offset
if (myAbs(Xdegree)>4)
{
Xd=Xdegree;
}
else
{
Xd=0;
}

pitch = pitch + Xd*dt;

// gyro Y offset
if (myAbs(Ydegree)>2)
{
Yd=Ydegree-1.212214;
}
else
{
Yd=0;
}

roll = roll + Yd*dt;

//roll + Yd*dt;

// gyro Z offset
if (myAbs(Zdegree)>2)
{
Zd=Zdegree-0.912214;
}
else
{
Zd=0;
}

yaw = yaw + Zd*dt;


fma=myAbs(pitchACC)+myAbs(rollACC)+myAbs(yawACC);

if(fma> 7)
{

// pitch,roll,yaw calculation

// complementary filter

pitch_cf = (pitch_cf + Xdegree*dt )* a + pitchACC * (1-a);
roll_cf = (roll_cf + Ydegree*dt) * a + rollACC * (1-a);
yaw_cf = (yaw_cf + Zdegree*dt) * a + yawACC * (1-a);
}


//printf(" \n Xh: %d, Xl: %d\n", Xh, Xl);
//printf(" Yh: %d, Yl: %d\n", Yh, Yl);
//printf(" Zh: %d, Zl: %d\n\n", Zh, Zl);
//printf(" Xgyro: %d, Ygyro: %d, Zgyro: %d\n",Xgyro,Ygyro,Zgyro);


// raw data
//printf("===============================================================\n");
printf(" Xdegree: %f, Ydegree: %f, Zdegree: %f    \n",Xdegree,Ydegree,Zdegree);
//printf(" Xd: %f, Yd: %f, Zd: %f    \n",Xd,Yd,Zd);
//printf("===============================================================\n");

printf(" pitchacc: %f, rollacc: %f, yawacc: %f    \n",pitchACC,rollACC,yawACC);
//printf (" fma: %f  \n",fma);
// aero data
//printf("===============================================================\n");
//printf(" pitch: %f, roll: %f, yaw: %f    \n",pitch,roll,yaw);
//printf("---------------------------------------------------------------\n");
printf(" pitch_cf: %f, roll_cf: %f, yaw_cf: %f    \n",pitch_cf,roll_cf,yaw_cf);
printf("===============================================================\n");

//put data to file
fprintf(dataFILE, "%d %f %f %f %f %f %f\n", i , pitchACC, pitch, pitch_cf, pitchACC, rollACC, yawACC); 
i++;
//gpio
digitalWrite(testPin,LOW);

//delay(500);
};


return 0;
}
/***************  control event  ************************/


void control_event(int sig){

printf("\n---------   EXITING   ---------\n");

//set gnuplot
fprintf(gnuplotPipe, " set title \"pitch\"\n"); 
fprintf(gnuplotPipe, "set xlabel \"data number\"\n");
fprintf(gnuplotPipe, "set ylabel \"value\" \n");

fprintf(gnuplotPipe1, " set title \"degree\"\n"); 
fprintf(gnuplotPipe1, "set xlabel \"data number\"\n");
fprintf(gnuplotPipe1, "set ylabel \"value\" \n");
//plot
fprintf(gnuplotPipe, "plot 'data.temp' u 1:2 t \"pitchACC\" with lines,");//not finish plotting 
fprintf(gnuplotPipe, "     'data.temp' u 1:3 t \"pitch\" with lines,"); 
fprintf(gnuplotPipe, "     'data.temp' u 1:4 t \"pitch_cf\" with lines\n"); 


/*
fprintf(gnuplotPipe1, "plot 'data.temp' u 1:5 t \"pitchACC\" with lines,"); 
fprintf(gnuplotPipe1, "     'data.temp' u 1:6 t \"rollACC\" with lines,"); 
fprintf(gnuplotPipe1, "     'data.temp' u 1:7 t \"yawACC\" with lines\n"); 
*/


fclose(dataFILE);
fflush(gnuplotPipe);
fflush(gnuplotPipe1);

exit(0);


}

