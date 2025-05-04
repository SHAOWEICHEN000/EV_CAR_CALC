#ifndef SOLAR_H
#define SOLAR_H
typedef struct{
double area;
double panelEff;
double dcdcEff;
double pSun;
}solar;
solar setupSolar(double area,double panelEff,double dcdcEff,double pSun);
#endif
