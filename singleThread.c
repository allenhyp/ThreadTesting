#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>


#define pi 3.14159265359
#define WEIGHT 0.98
#define ACC_THRESHOLD 5
#define TIMELIMIT 2.5
//gnuplot
// #define NUM_POINTS 9999
// FILE * dataFILE;
// FILE * gnuplotPipe ;
// FILE * gnuplotPipe1 ;

#define testPin 2

//Global variables for threads
double lastTime, nowTime;
double cycleStartTime, restTime;
double dt;
int fd;
float pitch_cf = 0, roll_cf = 0, yaw_cf = 0;


float myAbs(float value){
   if(value>=0)
      return value;
   else
      return -value;
}

double getCurrentTime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double sec = tv.tv_sec*1000 + tv.tv_usec/1000;
    return sec;
}

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
   short xValue, yValue, zValue;
   float xDegree, yDegree, zDegree;
   float Xg, Yg, Zg, pa, ra, ya;
   float Xd, Yd, Zd;
   float pitchACC, rollACC, yawACC;
   float pitch = 0, roll = 0, yaw = 0;
   float fma;
   int i = 0;
   //Start the loop
   while(i<1000){
      cycleStartTime = getCurrentTime();
      //Get and Set Acc data
      //********************
      Xgh = wiringPiI2CReadReg8(fd, 0x3B);
      Xgl = wiringPiI2CReadReg8(fd, 0x3C);

      Ygh = wiringPiI2CReadReg8(fd, 0x3D);
      Ygl = wiringPiI2CReadReg8(fd, 0x3E);

      Zgh = wiringPiI2CReadReg8(fd, 0x3F);
      Zgl = wiringPiI2CReadReg8(fd, 0x40);

      xValue = Xgh*256 + Xgl;
      yValue = Ygh*256 + Ygl;
      zValue = Zgh*256 + Zgl;
      Xg = (float)xValue / 16384;
      Yg = (float)yValue / 16384;
      Zg = (float)zValue / 16384;

      pa = sqrt(Xg*Xg + Zg*Zg);
      ra = sqrt(Yg*Yg + Zg*Zg);
      ya = sqrt(Xg*Xg + Yg*Yg);

      pitchACC = atan(Yg/pa)*180/pi;
      rollACC = atan(Xg/ra)*180/pi;
      yawACC = atan(ya/Zg)*180/pi;

      //Get and Set Gyro data
      //*********************
      Xh = wiringPiI2CReadReg8(fd, 0x43);
      Xl = wiringPiI2CReadReg8(fd, 0x44);
      xDegree = (float)(Xh * 256 + Xl) / 131;

      Yh = wiringPiI2CReadReg8(fd, 0x45);
      Yl = wiringPiI2CReadReg8(fd, 0x46);
      yDegree = -(float)(Yh * 256 + Yl) / 131;

      Zh = wiringPiI2CReadReg8(fd, 0x47);
      Zl = wiringPiI2CReadReg8(fd, 0x48);
      zDegree = (float)(Zh * 256 + Zl) / 131;

      if (myAbs(xDegree)>4) {
         Xd=xDegree+3.412214;
      }
      else {
         Xd=0;
      }

      // gyro Y offset
      if (myAbs(yDegree)>2) {
         Yd=yDegree-1.342214;
      }
      else {
         Yd=0;
      }

      // gyro Z offset
      if (myAbs(zDegree)>2) {
         Zd=zDegree-0.912214;
      }
      else {
         Zd=0;
      }
      //Get time eclipsed
      nowTime - getCurrentTime();
      dt = ((double)nowTime - lastTime)/1000;
      lastTime = nowTime;

      fma = myAbs(pitchACC) + myAbs(rollACC) + myAbs(yawACC);
      
      pitch = pitch + Xd * dt;
      roll = roll + Yd * dt;
      yaw = yaw + Zd * dt;

      if (fma > ACC_THRESHOLD){
         pitch_cf = (pitch_cf + xDegree * dt) * WEIGHT + pitchACC * (1 - WEIGHT);
         roll_cf = (roll_cf - yDegree * dt) * WEIGHT + rollACC * (1 - WEIGHT);
         yaw_cf = (yaw_cf + zDegree * dt) * WEIGHT + yawACC * (1 - WEIGHT);
      }
      

      //Print the result
      // printf("pitchACC: %f, rollACC: %f, yawACC: %f.\n", pitchACC, rollACC, yawACC);
      // printf("pitch: %f, roll: %f, yaw: %f.\n", pitch, roll, yaw);
      // printf("pitch_cf: %f, roll_cf: %f, yaw_cf: %f.\n", pitch_cf, roll_cf, yaw_cf);

      //Sleep for the rest of time
      restTime = cycleStartTime - getCurrentTime();
      if (restTime < TIMELIMIT){
         usleep(restTime);
      }
      i++;
      // else{
      //    printf("Timeout!\n");
      // }
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
   //ret = pthread_attr_setschedparam(&attrMain, (int) SCHED_FIFO);


   int i = 0;
   lastTime - getCurrentTime();
   pthread_create (&threadAlgo, &attrMain, calculationThread, NULL);
   while(1){
      //printf("Loop....\n");
      sleep(1);
   }
   return 0;
}