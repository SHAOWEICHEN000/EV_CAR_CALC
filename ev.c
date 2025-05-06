#include <stdio.h>
#include<stdint.h>
#include<stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <stdbool.h>
#include <limits.h>
#include <float.h>
#include "solar.h"
#include"battery.h"
#include "mechanic.h"
#include"vehicle.h"
#include"throttle.h"
//#include "plot_bridge.h"
#include "define.h"
#include"free.h"
#include"memalloc.h"
#include"power.h"
double t[MAX_LINES], alpha[MAX_LINES], beta[MAX_LINES];
double v[MAX_LINES],usable,ran_Km;
int n,iter;
int main() {

    //-----------------------------------------------------------------Set up parameter-----------------------------------------------------------------
    Cell c=setupCell(CELL_CAPACITY,CELL_WEIGHT,CELL_VMAX,CELL_VNOM,CELL_VMIN);
    module_cell m= setupModule(MODULE_P, MODULE_S, MODULE_OVHD, c);
    pack battery=setupPack(PACK_S,PACK_OVHD,PACK_SOCF,PACK_SOCE,PACK_ETA,m);
    wheel wheel=setupWheel(WHEEL_RADIUS,WHEEL_INERTIA,WHEEL_ROLL);
    motor motor=setupMotor(MOTOR_LMAX,MOTOR_RPMR,MOTOR_RPMMAX, MOTOR_ETA,MOTOR_INERTIA);
    dt    driveTr=setupDriveTrain(DRIVE_TRAIN_INVEFF,DRIVE_TRAIN_REGEN,DRIVE_TRAIN_RATIO,DRIVE_TRAIN_GEARJ,DRIVE_TRAINGEAREFF, battery,motor,wheel);
    solar solar=setupSolar(SOLAR_AREA,SOLAR_PANELEFF,SOLAR_DCDCEFF,SOLAR_PSUN);
    veh   vehicle=setupVehicle(VEH_WHEELS,VEH_ROADF,VEH_CD,VEH_A,VEH_WEIGHT,VEH_PAYLOAD,VEH_OVERPWR,driveTr,VEH_WIND,VEH_ANGLE);
    readThrottleData("throttle.txt", t, alpha, beta, MAX_LINES, &n);
    //-----------------------------------------------------------recall function-------------------------------------------------------------------------
    double *v = malloc(n*sizeof(double));
    double *a=malloc(n*sizeof(double));
    double *Td=malloc(n*sizeof(double));
    double *Tm=malloc(n*sizeof(double));
    double *motRPM=malloc(n*sizeof(double));
    double *SOC=malloc(n*sizeof(double));
    double *Pbat=malloc(n*sizeof(double));
    double *Vin=malloc(n*sizeof(double));
    double *dist=malloc(n*sizeof(double));
    double *G=malloc(n*sizeof(double));
    double *Ibat=malloc(n*sizeof(double));
    double *Pmot=malloc(n*sizeof(double));
    memalloc(v,a, Td,Tm,motRPM,SOC,dist,Pbat,Vin,G,Ibat,Pmot);//check memory alloc condition
   
    //------------------------------------------------------------------main----------------------------------------------------------------------------
    SOC[0]=battery.socFull;
    v[0]=INITIAL_VELEOCITY;
    a[0]=0;
    motRPM[0]=0.0;
    Tm[0]=0.0;
    G[0]=0;
    Ibat[0]=0;
    for(int i=1;i<n;i++)
    {
    SOC[i]=battery.socFull;
    Vin[i]=alphaRatio(alpha,battery,i);
    motRpmCalc(v,wheel,driveTr,i,motRPM);
    judgeTorque(i,motRPM,motor,Vin,battery,driveTr,beta,alpha,Tm);
    v[i]=velocityDetect(Tm,driveTr,wheel,aeroForce(vehicle,v[i-1]),rollForce(wheel,vehicle,v[i-1]),gradeForce(vehicle),brakeForce(vehicle),vehicle,a,v,i);
    Pmot[i]  = motPower(motRPM,Tm,i);  //kW
    G[i] = solarPower(solar);   // kW   
    PowerToSoc(Pmot,G,i,vehicle,driveTr,Ibat,SOC,battery,Pbat);//Power and SOC
    dist[i]= dist[i-1] + v[i]*time/1000; // km   
    iter=i;   	
    }
     usable = vehicle.driveTr.battery.socFull - vehicle.driveTr.battery.socEmpty;
     ran_Km = dist[iter] * usable/(SOC[0]-SOC[iter]+eps);
    
    //-----------------------------------------------------------------result----------------------------------------------------------------------------
    printf("Drive_train_efficiency=%f\n",driveTr.efficiency);
    printf("max speed=%f \n",vehicle.maxSpeed);
    printf("SOC STATUS=%f \n",SOC[0]);
    printf("Vin[%d]=%lf\n",0,Vin[0]);
    printf("Tm=%lf\n",beta[2]);
    printf("Motor_Lmax=%lf\n",motor.Lmax);
    printf("battery.Vnom=%lf\n",battery.Vnom);
    printf("usable ≈ %5.1f %\n", usable);
    printf("Range ≈ %5.1f km\n", ran_Km);
    for (int i = 0; i < n; i++) {
        printf("Line %2d: Time = %8.2f, Throttle = %8.2f, Brake = %8.2f, Vin = %8.2f, v = %8.2f, a = %8.2f, Tm = %8.2f, motRPM=%8.2f,Pmot=%8.2f,Pbat=%8.2f,SOC=%8.2f,dist=%8.2f\n",
       i + 1, t[i], alpha[i], beta[i], Vin[i], v[i], a[i], Tm[i],motRPM[i],Pmot[i],Pbat[i],SOC[i],dist[i]);
        
       //printf("v(%d)=%lf\n",i,v[i]);
    }
    //-----------------------------------------------------------------free pointer----------------------------------------------------------------------
    freeMem(v,a, Td,Tm,motRPM,SOC,dist,Pbat,Vin,G,Ibat,Pmot);
    return 0;
}

