#include <stdio.h>
#include"define.h"
#include"battery.h"
#include"mechanic.h"
#include"solar.h"
#include"free.h"
#include"memalloc.h"
#include"vehicle.h"
#include "throttle.h"
#include"power.h"

double motPower(double *motRPM,double *Tm,int i){

return (motRPM[i]+motRPM[i-1])/2 * 2*pi/60 * Tm[i] / (1000.0);  // kW

}

double solarPower(solar solar){

return solar.pSun*solar.area*solar.panelEff*solar.dcdcEff;

}

void   PowerToSoc(double *Pmot,double *G,int i,veh vehicle,dt driveTr,double *Ibat,double *SOC,pack battery,double *Pbat){
	    if (Pmot[i] >= 0){
                Pbat[i]= vehicle.overHeadPwr+ Pmot[i]/driveTr.efficiency-G[i];//Accerlate
                }
            else {
                Pbat[i]= vehicle.overHeadPwr+ Pmot[i]*driveTr.efficiency-G[i];//declate
            } 
            Ibat[i] =1000* Pbat[i]/(battery.Vnom);                     // A
            SOC[i]= SOC[i-1] - Ibat[i]*time / (3600*battery.capacity)*100;
            if(SOC[i]>100)
                SOC[i]=100;
             else if(SOC[i]<0)
                SOC[i]=0;
          
}
