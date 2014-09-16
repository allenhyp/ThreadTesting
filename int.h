
float function();
int fd;
#define pi 3.14159265359

float myAbs(float value){
   if(value>=0)
      return value;
   else
      return -value;
}

float function(){
fd = wiringPiI2CSetup (0x68);
wiringPiI2CWriteReg8 (fd,0x6B,0x01); //set the sleep unenable,chose clock source.
wiringPiI2CWriteReg8 (fd,0x19,0x00); //set sample rate divide = 0
wiringPiI2CWriteReg8 (fd,0x1A,0x00); //set gyroscope output rate = 8khz

printf("set 0x6B=%Xn",wiringPiI2CReadReg8 (fd,0x6B));

unsigned char Xgh,Xgl,Ygh,Ygl,Zgh,Zgl, Xh,Xl,Yh,Yl,Zh,Zl;
short Xvalue, Yvalue, Zvalue, Xgyro, Ygyro, Zgyro;
float Xg, Yg, Zg, Xdegree, Ydegree, Zdegree, pa, ra, ya, Xd, Yd, Zd, pitchACC;
float  roll, yaw, rollACC, yawACC,  roll_cf, yaw_cf;

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

if (myAbs(pitchACC)<4)
{
pitchACC =0;
}

return pitchACC;
}
