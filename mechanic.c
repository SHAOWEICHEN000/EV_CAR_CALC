#include"battery.h"
#include"mechanic.h"
#include"define.h"
wheel setupWheel(double r,double inertia,double roll){
wheel w;
w.radius=r;
w.inertia=inertia;
w.rollCoef=roll;
return w;
}

motor setupMotor(double Lmax,double RPMr,double RPMmax,double eta,double inertia)
{
motor mt;
mt.Lmax=Lmax;
mt.RPMrated=RPMr;
mt.RPMmax=RPMmax;
mt.efficiency=eta;
mt.inertia=inertia;
mt.maxPower=2*pi*Lmax*RPMr/60000;	//KW
return mt;
}

dt setupDriveTrain(double invEff,double regen, double ratio,double gearJ,double gearEff,pack battery,motor motor,wheel wheel)
{
dt dt;
dt.invertEfficiency=invEff;
dt.regenTorque=regen;
dt.gearRatio=ratio;
dt.gearInertia=gearJ;
dt.gearEfficiency=gearEff;
dt.battery=battery;
dt.motor=motor;
dt.wheel=wheel;
dt.efficiency=battery.efficiency*invEff*motor.efficiency*gearEff;
return dt;
}
