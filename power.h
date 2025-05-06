#ifndef POWER_H
#define POWER_H
double motPower(double *motRPM,double *Tm,int i);
double solarPower(solar solar);
void   PowerToSoc(double *Pmot,double *G,int i,veh vehicle,dt driveTr,double *Ibat,double *SOC,pack battery,double *Pbat);
#endif
