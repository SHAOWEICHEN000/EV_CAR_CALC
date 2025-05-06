#include <stdio.h>
#include"define.h"
#include"battery.h"
#include"mechanic.h"
#include"free.h"
#include"memalloc.h"
#include"vehicle.h"
#include "throttle.h"

void readThrottleData(const char *filename, double *t, double *alpha, double *beta, int maxLine, int *n) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("Cannot open file");
        *n = 0;
        return;
    }

    // 跳過前兩行
    char skip[256];
    fgets(skip, sizeof(skip), fp);
    fgets(skip, sizeof(skip), fp);

    int i = 0;
    while (i < maxLine && fscanf(fp, "%lf%lf%lf", &t[i], &alpha[i], &beta[i]) == 3) {
        i++;
    }

    *n = i;
    fclose(fp);
}

double alphaRatio(double *alpha,pack battery,int i){
    double Vin;
    Vin=alpha[i]/100*battery.Vnom;
    return Vin;
}

void motRpmCalc(double *v,wheel wheel ,dt driveTr,int i,double *motRPM)
{  
   //printf("%lf\n",v[i]);
   motRPM[i]=v[i-1]/wheel.radius*driveTr.gearRatio*60/(2*pi);
}

double minCom(double a, double b) {
    return (a < b) ? a: b;
}

double maxCom(double a, double b) {
    return (a > b) ? a : b;
}

void   judgeTorque(int i,double *motRPM,motor motor,double *Vin,pack battery,dt driveTr,double *beta,double *alpha,double *Tm)
{
	double TmaxAvail,TregenLimit ,TminAvail ;
    if (motRPM[i]< motor.RPMrated)
                TmaxAvail = motor.Lmax * Vin[i] / (battery.Vnom);  // 線性降額
                
            else
                TmaxAvail = motor.Lmax * motor.RPMrated / motRPM[i] * Vin[i] / (battery.Vnom);
           
    //------------------------------------------------------------------------允許的再生扭矩-----------------------------------------------------------------
            TregenLimit = driveTr.regenTorque * motor.Lmax;            // Nm
            TminAvail   = - beta[i]/100 * TregenLimit;                 //Nm (負值)
            
            Tm[i] = 0;
            if ((alpha[i] >= 0)&&(beta[i]==0)){
                Tm[i] =  minCom( TmaxAvail ,  alpha[i]/100.0 * motor.Lmax); //DRIVE
                }
            else if(beta[i]  > 0){
                Tm[i] =  maxCom( TminAvail , -beta[i]/100.0 * motor.Lmax);  //REGEN
            }
            
}

double aeroForce(veh vehicle,double v){
return 0.5*rho*vehicle.Cd*vehicle.A*(v-vehicle.wind);
}
double rollForce(wheel wheel,veh vehicle,double v){
return wheel.rollCoef*vehicle.weight*g*(v);
}

double brakeForce(veh vehicle){
return vehicle.brakeDrag;
}
double gradeForce(veh vehicle)
{
return vehicle.roadForce;
}

double velocityDetect(double *Tm,dt driveTr,wheel wheel,double F_aero,double F_roll,double F_grade,double  F_brake,veh vehicle,double *a,double*v,int i){
double F_avail,velocity;
    F_avail=Tm[i]*driveTr.gearRatio / wheel.radius - F_aero - F_roll - F_grade-F_brake;
    a[i] = F_avail / vehicle.equivMass;
    
    v[i] = maxCom(0, v[i-1] + a[i]*time);   
    if (v[i] > vehicle.maxSpeed * 60.0 / 1000.0)
    	v[i] = vehicle.maxSpeed * 60.0 / 1000.0;
    	//printf("%lf\n",v[i-1] + a[i]*time);
    	return v[i];
}
