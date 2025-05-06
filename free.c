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
#include "define.h"
#include"free.h"
void freeMem(double *v,double *a,double *Td,double *Tm,double *motRPM,double *SOC,double *dist,double *Pbat,double *Vin,double *G,double *Ibat, double *Pmot)
{
    if (v)free(v);
    if (a)free(a);
    if (Td)free(Td);
    if (Tm)free(Tm);
    if (motRPM)free(motRPM);
    if (SOC)free(SOC);
    if (dist)free(dist);
    if (Pbat)free(Pbat);
    if (Vin)free(Vin);
    if (G)free(G);
    if(Ibat)free(Ibat);
    if(Pmot)free(Pmot);
}
