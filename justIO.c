#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>


#define pi 3.14159265359
#define WEIGHT 0.98
#define ACC_THRESHOLD 5
#define TIMELIMIT 2.5
#define COUNT 1000
//gnuplot
// #define NUM_POINTS 9999
// FILE * dataFILE;
// FILE * gnuplotPipe ;
// FILE * gnuplotPipe1 ;

// #define testPin 2

//Global variables for threads
// double lastTime, nowTime;
// double cycleStartTime, restTime;
// double dt;
// int fd;
// float pitch_cf = 0, roll_cf = 0, yaw_cf = 0;


// float myAbs(float value){
//    if(value>=0)
//       return value;
//    else
//       return -value;
// }

// double getCurrentTime(){
//     struct timeval tv;
//     gettimeofday(&tv, NULL);
//     double sec = tv.tv_sec*1000 + tv.tv_usec/1000;
//     return sec;
// }

// void threasholdOfDegree (float& X, float& Y, float& Z){
//    float tempX = X, tempY = Y, tempZ = Z; 
//    if (myAbs(tempX)>4) {
//       X=tempX+3.412214;
//    }
//    else {
//       X=0;
//    }

//    // gyro Y offset
//    if (myAbs(tempY)>2) {
//       Y=tempY-1.342214;
//    }
//    else {
//       Y=0;
//    }

//    // gyro Z offset
//    if (myAbs(tempZ)>2) {
//       Z=tempZ-0.912214;
//    }
//    else {
//       Z=0;
//    }
//    return;
// }

////Variable for thread
//*********************

void* calculationThread(void* arg){
   unsigned static char Xgh,Xgl,Ygh,Ygl,Zgh,Zgl,Xh,Xl,Yh,Yl,Zh,Zl;
   // short xValue, yValue, zValue;
   // float xDegree, yDegree, zDegree;
   // float Xg, Yg, Zg, pa, ra, ya;
   // float Xd, Yd, Zd;
   // float pitchACC, rollACC, yawACC;
   // float pitch = 0, roll = 0, yaw = 0;
   // float fma;
   int i = 0;
   //Start the loop
   while(i<COUNT){
      //Get and Set Acc data
      //********************
      Xgh = wiringPiI2CReadReg8(fd, 0x3B);
      Xgl = wiringPiI2CReadReg8(fd, 0x3C);

      Ygh = wiringPiI2CReadReg8(fd, 0x3D);
      Ygl = wiringPiI2CReadReg8(fd, 0x3E);

      Zgh = wiringPiI2CReadReg8(fd, 0x3F);
      Zgl = wiringPiI2CReadReg8(fd, 0x40);
      

      //Get and Set Gyro data
      //*********************
      Xh = wiringPiI2CReadReg8(fd, 0x43);
      Xl = wiringPiI2CReadReg8(fd, 0x44);

      Yh = wiringPiI2CReadReg8(fd, 0x45);
      Yl = wiringPiI2CReadReg8(fd, 0x46);

      Zh = wiringPiI2CReadReg8(fd, 0x47);
      Zl = wiringPiI2CReadReg8(fd, 0x48);

      i++;
   }
}

int main(){
   //Setup IO
   fd = wiringPiI2CSetup (0x68);
   wiringPiI2CWriteReg8 (fd,0x6B,0x01); //set the sleep unenable,chose clock source.
   wiringPiI2CWriteReg8 (fd,0x19,0x01); //set sample rate divide = 0
   wiringPiI2CWriteReg8 (fd,0x1A,0x01); //set gyroscope output rate = 8khz

   //Setup pthread
   int ret;
   pthread_t threadAlgo;
   pthread_attr_t attrMain;
   struct sched_param parmMain;
   pthread_attr_init(&attrMain);
   pthread_attr_setscope(&attrMain, PTHREAD_SCOPE_SYSTEM);
   parmMain.sched_priority = sched_get_priority_max (SCHED_FIFO);
   ret = pthread_attr_setschedparam(&attrMain, (int) SCHED_FIFO);


   // lastTime - getCurrentTime();
   pthread_create (&threadAlgo, &attrMain, calculationThread, NULL);
   printf("Started.\n");
   while(!pthread_join(threadAlgo, NULL)){
      //printf("ing\n");
   }
   printf("ENDED\n");
   pthread_exit(&threadAlgo);
   return 0;
}