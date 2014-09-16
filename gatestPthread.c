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
#define a 0.98

//gnuplot
// #define NUM_POINTS 9999
// FILE * dataFILE;
// FILE * gnuplotPipe ;
// FILE * gnuplotPipe1 ;

#define testPin 2

//Global variables for threads
unsigned static char Xgh,Xgl,Ygh,Ygl,Zgh,Zgl,Xh,Xl,Yh,Yl,Zh,Zl;
double dt;
int fd;
int terminal = 0;




void control_event(int sig);

float myAbs(float value){
   if(value>=0)
      return value;
   else
      return -value;
}

long getCurrentTime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000 + tv.tv_usec/1000;
}
//Variable for thread
//*******************
struct value{
   short Xv;
   short Yv;
   short Zv;
};

struct degree{
   float Xdg;
   float Ydg;
   float Zdg;
};

pthread_mutex_t mutexReadingAcc;
pthread_mutex_t mutexReadingGyro;
int readingAcc = 0;
int readingGyro = 0;

//Function for thread
//*******************
void* getAccData(void *v){
   while(!terminal){
         struct value *tempV = v;
         pthread_mutex_lock(&mutexReadingAcc);
         readingAcc = 1;
         Xgh = wiringPiI2CReadReg8(fd, 0x3B);
         Xgl = wiringPiI2CReadReg8(fd, 0x3C);

         Ygh = wiringPiI2CReadReg8(fd, 0x3D);
         Ygl = wiringPiI2CReadReg8(fd, 0x3E);

         Zgh = wiringPiI2CReadReg8(fd, 0x3F);
         Zgl = wiringPiI2CReadReg8(fd, 0x40);

         tempV->Xv = Xgh*256 + Xgl;
         tempV->Yv = Ygh*256 + Ygl;
         tempV->Zv = Zgh*256 + Zgl;

         
         readingAcc = 0;
         pthread_mutex_unlock(&mutexReadingAcc);
         printf("Received data from acc.\n");
         usleep(100);
      }
      
      return;
}
void* getGyroData(void *d){
   while(!terminal){
         struct degree *tempD = d;
         pthread_mutex_lock(&mutexReadingGyro);
         readingGyro = 1;
         Xh = wiringPiI2CReadReg8(fd, 0x43);
         Xl = wiringPiI2CReadReg8(fd, 0x44);
         tempD->Xdg = (float)(Xh * 256 + Xl) / 131;

         Yh = wiringPiI2CReadReg8(fd, 0x45);
         Yl = wiringPiI2CReadReg8(fd, 0x46);
         tempD->Ydg = -(float)(Yh * 256 + Yl) / 131;

         Zh = wiringPiI2CReadReg8(fd, 0x47);
         Zl = wiringPiI2CReadReg8(fd, 0x48);
         tempD->Zdg = (float)(Zh * 256 + Zl) / 131;
         
         readingGyro = 0;
         pthread_mutex_unlock(&mutexReadingGyro);
         printf("Received data from gyro.\n");
         printf("now Xdg: %f\n", tempD->Xdg);
         usleep(100);
   }
   
   return;
}

int main(){
// ctl + c ...exiting
   // (void)signal(SIGINT,control_event);
   // (void)signal(SIGQUIT,control_event);
   printf("Start!!\n");
   fd = wiringPiI2CSetup (0x68);
   wiringPiI2CWriteReg8 (fd,0x6B,0x01); //set the sleep unenable,chose clock source.
   wiringPiI2CWriteReg8 (fd,0x19,0x01); //set sample rate divide = 0
   wiringPiI2CWriteReg8 (fd,0x1A,0x01); //set gyroscope output rate = 8khz
   // printf("set 0x6B=%Xn",wiringPiI2CReadReg8 (fd,0x6B));

   short Xvalue, Yvalue, Zvalue, Xgyro, Ygyro, Zgyro;
   float Xg, Yg, Zg, Xdegree, Ydegree, Zdegree, pa, ra, ya, Xd, Yd, Zd;
   float pitch, roll, yaw, pitchACC, rollACC, yawACC;
   float pitch_cf=0, roll_cf=0, yaw_cf=0;
   float fma;
   long lastTime, nowTime;
   double tempDt;

   struct value valueStruct;
   struct degree degreeStruct;
   pthread_t threadAcc;
   pthread_t threadGyro;
   pthread_mutex_init(&mutexReadingAcc, NULL);
   pthread_mutex_init(&mutexReadingGyro, NULL);
   //gpio
   // wiringPiSetup();
   // pinMode(testPin,OUTPUT);
   // digitalWrite(testPin,LOW);

   // //file
   int i = 0;
   // dataFILE = fopen("data.temp", "w");
   // gnuplotPipe = popen ("gnuplot -persistent", "w");
   // gnuplotPipe1 = popen ("gnuplot -persistent", "w");
   printf("Creating thread.\n");
   pthread_create (&threadAcc, NULL, getAccData, (void *)&valueStruct);
   pthread_create (&threadGyro, NULL, getGyroData, (void *)&degreeStruct);
   lastTime = getCurrentTime();

   printf("Getting in to the while loop.%d\n", lastTime);
   while(1){
      
      
      /*while(readingAcc==1||readingGyro==1){
            printf("Waiting for reading\n");
      }*/

      //acc
      //********************************
      //thread parameter: Xvalue, Yvalue, Zvalue --> valueStruct

      Xg = (float)valueStruct.Xv / 16384;
      Yg = (float)valueStruct.Yv / 16384;
      Zg = (float)valueStruct.Zv / 16384;

      pa = sqrt(Xg*Xg + Zg*Zg);
      ra = sqrt(Yg*Yg + Zg*Zg);
      ya = sqrt(Xg*Xg + Yg*Yg);

      pitchACC = atan(Yg/pa)*180/pi;
      rollACC = atan(Xg/ra)*180/pi;
      yawACC = atan(ya/Zg)*180/pi;


      // gyro
      //*********************************
      //thread parameter: Xdegree, Ydegree, Zdegree
      Xdegree = degreeStruct.Xdg;
      Ydegree = degreeStruct.Ydg;
      Zdegree = degreeStruct.Zdg;
      // gyro X offset
      if (myAbs(Xdegree)>4) {
         Xd=Xdegree+3.412214;
      }
      else {
         Xd=0;
      }

      // gyro Y offset
      if (myAbs(Ydegree)>2) {
         Yd=Ydegree-1.342214;
      }
      else {
         Yd=0;
      }

      // gyro Z offset
      if (myAbs(Zdegree)>2) {
         Zd=Zdegree-0.912214;
      }
      else {
         Zd=0;
      }
      
      nowTime = getCurrentTime();
      dt = ((double)nowTime-lastTime)/1000;
      lastTime = nowTime;
      
      printf("Time eclipsed(s): %f\n", dt);
      pitch = pitch + Xd*dt;
      roll = roll + Yd*dt;
      yaw = yaw + Zd*dt;


      fma=myAbs(pitchACC)+myAbs(rollACC)+myAbs(yawACC);

      if(fma> 5)
      {

      // pitch,roll,yaw calculation

      // complementary filter

         pitch_cf = (pitch_cf + Xdegree*dt )* a + pitchACC * (1-a);
         roll_cf = (roll_cf - Ydegree*dt) * a + rollACC * (1-a);
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
      //printf(" pitch_cf: %f, roll_cf: %f, yaw_cf: %f    \n",pitch_cf,roll_cf,yaw_cf);
      printf("===============================================================\n");

      //put data to file
      //fprintf(dataFILE, "%d %f %f %f %f %f %f\n", i , rollACC, roll, roll_cf, pitchACC, pitch, pitch_cf); 
      i++;
      //gpio
      digitalWrite(testPin,LOW);
      usleep(100);
      //delay(500);
   }
   //pthread_mutex_destroy(&readingAcc);
   //pthread_mutex_destroy(&readingGyro);
   terminal = 1;
   printf("End of the program.\n");
   pthread_exit(&threadAcc);
   pthread_exit(&threadGyro);
   return 0;
}

/***************  control event  ************************/


// void control_event(int sig){

// printf("\n---------   EXITING   ---------\n");

// //set gnuplot
// fprintf(gnuplotPipe, " set title \"roll\"\n"); 
// fprintf(gnuplotPipe, "set xlabel \"data number\"\n");
// fprintf(gnuplotPipe, "set ylabel \"value\" \n");

// fprintf(gnuplotPipe1, " set title \"pitch\"\n"); 
// fprintf(gnuplotPipe1, "set xlabel \"data number\"\n");
// fprintf(gnuplotPipe1, "set ylabel \"value\" \n");
// //plot
// fprintf(gnuplotPipe, "plot 'data.temp' u 1:2 t \"rollACC\" with lines,");//not finish plotting 
// fprintf(gnuplotPipe, "     'data.temp' u 1:3 t \"roll\" with lines,"); 
// fprintf(gnuplotPipe, "     'data.temp' u 1:4 t \"roll_cf\" with lines\n"); 


// /*
// fprintf(gnuplotPipe1, "plot 'data.temp' u 1:5 t \"pitchACC\" with lines,"); 
// fprintf(gnuplotPipe1, "     'data.temp' u 1:6 t \"pitch\" with lines,"); 
// fprintf(gnuplotPipe1, "     'data.temp' u 1:7 t \"pitch_cf\" with lines\n"); 
// */


// fclose(dataFILE);
// fflush(gnuplotPipe);
// fflush(gnuplotPipe1);

// exit(0);


// }

