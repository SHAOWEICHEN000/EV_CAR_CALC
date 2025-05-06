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
#include"memalloc.h"
void memalloc(double *v,double *a,double *Td,double *Tm,double *motRPM,double *SOC,double *dist,double *Pbat,double *Vin,double *G,double *Ibat,double *Pmot)
{
    if (!v) {
    perror("Memory allocation failed");
    exit(1);
    }
    if(!a){
    perror("memory allocation failed");
    exit(1);
    }
    if(!Td){
    perror("memory allocation failed");
    exit(1);
    }
    if(!Tm){
    perror("memory allocation failed");
    exit(1);
    }
    if(!motRPM){
    perror("memory allocation failed");
    exit(1);
    }   
    if(!SOC){
    perror("memory allocation failed");
    exit(1);
    }    
    if(!dist){
    perror("memory allocation failed");
    exit(1);
    }  
    if(!Pbat){
    perror("memory allocation failed");
    exit(1);
    }
    if(!Vin){
    perror("memory allocation failed");
    exit(1);
    }
    if(!G){
    perror("memory allocation failed");
    exit(1);
    }
    if(!Ibat){
    perror("memory allocation failed");
    exit(1);
    }  
     if(!Pmot){
    perror("memory allocation failed");
    exit(1);
    }   
}
