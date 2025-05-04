#ifndef BATTERY_H
#define BATTERY_H
typedef struct{
double capacity;
double weight;
double Vmax;
double Vmin;
double Vnom;
double energy;
double specificEnergy;
}Cell;

typedef struct{
double numParallel;
double numSeries;
double overHead;
Cell   cell;
double numCells;
double capacity;
double weight;
double energy;
double specificEnergy;
}module_cell;

typedef struct{
double numSeries;
double overHead;
module_cell m;
double socFull;
double socEmpty;
double efficiency;
double numCells;
double weight;
double energy;
double specificEnergy;
double Vmax;
double Vmin;
double capacity;
double Vnom;
}pack;

Cell setupCell(double cap,double wt,double vmax,double vnom,double vmin);
module_cell setupModule(double p,double s,double ovhd,Cell cell); 
pack setupPack(double s, double ovhd,double socF,double socE,double eta, module_cell m);
#endif
