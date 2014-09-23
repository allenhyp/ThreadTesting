#ifndef PLOT_H
#define PLOT_H

#include <stdio.h>
#include <stdlib.h>

//gnuplot
FILE * dataFILE;//put data here

FILE * gnuplotPipe ;//height
FILE * gnuplotPipe1 ;//pitch
FILE * gnuplotPipe2 ;//roll
FILE * gnuplotPipe3 ;//yaw
FILE * gnuplotPipe4 ;//filter

void gnuPlotOpenfile(){
	dataFILE = fopen("data.temp", "w");

	gnuplotPipe = popen ("gnuplot -persistent", "w");
	gnuplotPipe1 = popen ("gnuplot -persistent", "w");
	gnuplotPipe2 = popen ("gnuplot -persistent", "w");
	gnuplotPipe3 = popen ("gnuplot -persistent", "w");
	gnuplotPipe4 = popen ("gnuplot -persistent", "w");

}

void gnuPlotPutToFile(int dataNum){
	fprintf(dataFILE, "%d  %f %f ", dataNum, baroH, QUADCOPTER.altitude); //height
	fprintf(dataFILE, "%f  %f %f ", ACCEL.pitch, GYRO.pitch, QUADCOPTER.pitch); //pitch
	fprintf(dataFILE, "%f  %f %f ", ACCEL.roll, GYRO.roll, QUADCOPTER.roll); //roll
	fprintf(dataFILE, "%f  %f %f ", magZ, GYRO.yaw, QUADCOPTER.yaw); //yaw
	fprintf(dataFILE, "%f  %f %f \n", QUADCOPTER.pitch, QUADCOPTER.roll, QUADCOPTER.yaw); //filter	
}

void gnuPlot(){
	//set gnuplot
	fprintf(gnuplotPipe, " set title \"altitude\"\n"); 
	fprintf(gnuplotPipe, "set xlabel \"data number\"\n");
	fprintf(gnuplotPipe, "set ylabel \"height(m)\" \n");

	fprintf(gnuplotPipe1, " set title \"pitch\"\n"); 
	fprintf(gnuplotPipe1, "set xlabel \"data number\"\n");
	fprintf(gnuplotPipe1, "set ylabel \"degree\" \n");

	fprintf(gnuplotPipe2, " set title \"roll\"\n"); 
	fprintf(gnuplotPipe2, "set xlabel \"data number\"\n");
	fprintf(gnuplotPipe2, "set ylabel \"degree\" \n");

	fprintf(gnuplotPipe3, " set title \"yaw\"\n"); 
	fprintf(gnuplotPipe3, "set xlabel \"data number\"\n");
	fprintf(gnuplotPipe3, "set ylabel \"degree\" \n");

	fprintf(gnuplotPipe4, " set title \"quadcopter\"\n"); 
	fprintf(gnuplotPipe4, "set xlabel \"data number\"\n");
	fprintf(gnuplotPipe4, "set ylabel \"degree\" \n");
	
	///////plot
	//height
/*	fprintf(gnuplotPipe, "plot 'data.temp' u 1:2 t \"baro_H\" with lines,");
	fprintf(gnuplotPipe, "     'data.temp' u 1:3 t \"Quad_H\" with lines\n"); 
	//pitch
	fprintf(gnuplotPipe1, "plot 'data.temp' u 1:4 t \"ACCEL\" with lines,");
	fprintf(gnuplotPipe1, "     'data.temp' u 1:5 t \"GYRO\" with lines,"); 
	fprintf(gnuplotPipe1, "     'data.temp' u 1:6 t \"QUADCOPTER\" with lines\n"); 
	//roll
	fprintf(gnuplotPipe2, "plot 'data.temp' u 1:7 t \"ACCEL\" with lines,");
	fprintf(gnuplotPipe2, "     'data.temp' u 1:8 t \"GYRO\" with lines,"); 
	fprintf(gnuplotPipe2, "     'data.temp' u 1:9 t \"QUADCOPTER\" with lines\n"); 
*/	//yaw
	fprintf(gnuplotPipe3, "plot 'data.temp' u 1:10 t \"MAG\" with lines,");
	fprintf(gnuplotPipe3, "     'data.temp' u 1:11 t \"GYRO\" with lines,"); 
	fprintf(gnuplotPipe3, "     'data.temp' u 1:12 t \"QUADCOPTER\" with lines\n"); 
	//filter
	fprintf(gnuplotPipe4, "plot 'data.temp' u 1:13 t \"pitch\" with lines,");
	fprintf(gnuplotPipe4, "     'data.temp' u 1:14 t \"roll\" with lines,"); 
	fprintf(gnuplotPipe4, "     'data.temp' u 1:15 t \"yaw\" with lines\n"); 

	fclose(dataFILE);
	fflush(gnuplotPipe);
	fflush(gnuplotPipe1);
	fflush(gnuplotPipe2);
	fflush(gnuplotPipe3);
	fflush(gnuplotPipe4);
}

#endif
