#include <stdio.h>
#include"define.h"
#include"battery.h"
#include"mechanic.h"
#include"free.h"
#include"memalloc.h"
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
    Vin=alpha[i]/1000*battery.Vnom;
    return Vin;
}

double motRpmCalc(double Vin,wheel wheel ,dt driveTr)
{
    printf("v(0)=%lf\n",Vin);
    double motRpm1=Vin/wheel.radius*driveTr.gearRatio*60/(2*pi);
    return motRpm1;
}

double minCom(double a, double b) {
    return (a < b) ? a : b;
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
            if ((alpha[i] >= 0)^(beta[i]==0)){
                Tm[i] =  minCom( TmaxAvail ,  alpha[i]/100 * motor.Lmax); //DRIVE
                }
            else if(beta[i]  > 0){
                Tm[i] =  maxCom( TminAvail , -beta[i]/100 * motor.Lmax);  //REGEN
            }
}
