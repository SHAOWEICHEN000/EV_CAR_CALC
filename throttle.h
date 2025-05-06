#ifndef THROTTLE_H
#define THROTTLE_H
void   readThrottleData(const char *filename, double *t, double *alpha, double *beta, int maxLine, int *n);
double alphaRatio(double *alpha,pack battery,int i);
void   motRpmCalc(double *v,wheel wheet,dt driveTr,int i,double *motRPM) ;
double maxCom(double a, double b) ;
double minCom(double a, double b) ;
void   judgeTorque(int i,double *motRPM,motor motor,double *Vin,pack battery,dt driveTr,double *beta,double *alpha,double *Tm);
double aeroForce(veh vehicle,double v);
double rollForce(wheel wheel,veh vehicle,double v);
double brakeForce(veh vehicle);
double gradeForce(veh vehicle);
double velocityDetect(double *Tm,dt driveTr,wheel wheel,double F_aero,double F_roll,double F_grade,double  F_brake,veh vehicle,double *a,double *v,int i);
#endif
