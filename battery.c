#include"battery.h"
#include"define.h"
Cell setupCell(double cap, double wt, double vmax, double vnom, double vmin) {
    Cell c;
    c.capacity = cap;
    c.weight = wt;
    c.Vmax = vmax;
    c.Vnom = vnom;
    c.Vmin = vmin;
    c.energy = vnom * cap;
    c.specificEnergy = 1000 * cap * vnom / wt;
    return c;
}

module_cell setupModule(double p,double s,double ovhd,Cell cell){
    module_cell m;
    m.numParallel=p;
    m.numSeries=s;
    m.overHead=ovhd;
    m.cell=cell;
    m.numCells=p*s;
    m.capacity=p*cell.capacity;
    m.weight= m.numCells*cell.weight/(1-ovhd)/1000;//Kg
    m.energy=m.numCells*cell.energy/1000;	   //Kwh
    m.specificEnergy = 1000*m.energy/m.weight;	   //Wh/Kg
    return m;
}

pack setupPack(double s,double ovhd, double socF,double socE,double eta,module_cell m){
    pack p;	
    p.numSeries=s;
    p.overHead=ovhd;
    p.m=m;
    p.socFull=socF;
    p.socEmpty=socE;
    p.efficiency=eta;
    p.numCells=m.numCells*s;
    p.weight=m.weight*s/(1-ovhd);
    p.energy=m.energy*s;
    p.specificEnergy=1000*p.energy/p.weight;
    p.Vmax=s*m.cell.Vmax;
    p.Vnom=s*m.cell.Vnom;
    p.Vmin=s*m.cell.Vmin;
    p.capacity=m.capacity;
    return p;
};

