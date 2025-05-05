#ifndef THROTTLE_H
#define THROTTLE_H
void   readThrottleData(const char *filename, double *t, double *alpha, double *beta, int maxLine, int *n);
double alphaRatio(double *alpha,pack battery,int i);
double motRpmCalc(double Vin,wheel wheet,dt driveTr) ;
double maxCom(double a, double b) ;
double minCom(double a, double b) ;
void   judgeTorque(int i,double *motRPM,motor motor,double *Vin,pack battery,dt driveTr,double *beta,double *alpha,double *Tm);
#endif
