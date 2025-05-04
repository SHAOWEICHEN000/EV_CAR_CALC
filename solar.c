#include"define.h"
#include"solar.h"
solar setupSolar(double area,double panelEff,double dcdcEff,double pSun){
solar s;
s.area=area;
s.panelEff=panelEff;
s.dcdcEff=dcdcEff;
s.pSun=pSun;
return s;
}
