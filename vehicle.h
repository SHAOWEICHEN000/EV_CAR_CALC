#ifndef  VEHICLE_H
#define  VEHICLE_H
typedef struct{
dt driveTr;
double wheels;
double roadForce;
double Cd;
double A;
double weight;
double overHeadPwr;
double maxWeight;
double rotWeight;
double equivMass;
double maxSpeed;
double wind;
double brakeDrag;
}veh;

veh setupVehicle(double wheels,double roadF,double Cd,double A,double weight,double payload,double overPwr,dt driveTrain,double wind,double angle);
#endif
