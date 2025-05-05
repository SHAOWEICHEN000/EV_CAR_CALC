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
double t[MAX_LINES], alpha[MAX_LINES], beta[MAX_LINES];
double v[MAX_LINES];
int n;
double TmaxAvail,TregenLimit ,TminAvail ;
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
    memalloc(v,a, Td,Tm,motRPM,SOC,dist,Pbat,Vin);//check memory alloc condition
   
    //------------------------------------------------------------------main----------------------------------------------------------------------------
    SOC[0]=battery.socFull;
    v[0]=INITIAL_VELEOCITY;
    for(int i=1;i<n;i++)
    {
      
    //------------------------------------------------------------------ 由油門 α 計算可用馬達扭矩 ---------------------------------------------------------
    SOC[i]=battery.socFull;
    Vin[i]=alphaRatio(alpha,battery,i);
    motRPM[i]=motRpmCalc(v[i-1],wheel,driveTr);
    judgeTorque(i,motRPM,motor,Vin,battery,driveTr,beta,alpha,Tm);
    
            
    }
   
    
    //-----------------------------------------------------------------result----------------------------------------------------------------------------
    printf("Drive_train_efficiency=%f\n",driveTr.efficiency);
    printf("max speed=%f \n",vehicle.maxSpeed);
    printf("SOC STATUS=%f \n",SOC[0]);
    printf("Vin[%d]=%lf\n",0,Vin[0]);
    printf(" TmaxAvail=%lf\n",TmaxAvail);
    printf("TminAvail=%lf\n",TminAvail);
    printf("Tm=%lf\n",beta[2]);
    for (int i = 0; i < n; i++) {
       // printf("Line %d: Time = %.2f, Throttle = %.2f, Brake = %.2f\n", i + 1, t[i], alpha[i], beta[i]);
       //printf("Tm=%lf\n",motRPM[i]);
    }
    //-----------------------------------------------------------------free pointer----------------------------------------------------------------------
    freeMem(v,a, Td,Tm,motRPM,SOC,dist,Pbat,Vin);
    return 0;
}

