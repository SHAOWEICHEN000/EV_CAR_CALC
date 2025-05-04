#ifndef MECHANIC_H
#define MECHANIC_H

typedef struct{
double  radius;
double  inertia;
double  rollCoef;
}wheel; 

typedef struct{
double  Lmax;
double  RPMrated;
double  RPMmax;
double  efficiency;
double  inertia;
double  maxPower;
}motor;

typedef struct{
double 	invertEfficiency;
double  regenTorque;
double  gearRatio;
double  gearInertia;
double  gearEfficiency;
pack    battery;
motor   motor;
wheel   wheel;
double  efficiency;
}dt;

wheel setupWheel(double r,double inertia,double roll);
motor setupMotor(double Lmax,double RPMr,double RPMmax,double eta,double inertia);
dt    setupDriveTrain(double invEff,double regen,double ratio,double gearJ,double gearEff,pack battery, motor motor,wheel wheel);
#endif
