#include "define.h"
#include"battery.h"
#include"mechanic.h"
#include"vehicle.h"
#include<math.h>
veh setupVehicle(double wheels,double roadF,double Cd,double A,double weight,double payload,double overPwr,dt driveTr,double wind,double angle){
veh v;
v.wheels=wheels;
v.driveTr=driveTr;
v.roadForce=weight*g*sinl(angle);
v.Cd=Cd;
v.A=A;
v.weight=weight;
v.overHeadPwr=overPwr;
v.maxWeight=weight + driveTr.battery.weight + payload;
v.rotWeight= (driveTr.motor.inertia+driveTr.gearInertia)*pow(driveTr.gearRatio,2)/ pow(driveTr.wheel.radius,2)+ wheels*driveTr.wheel.inertia/pow(driveTr.wheel.radius,2);
v.equivMass=     v.maxWeight+ v.rotWeight;
v.maxSpeed=2*pi*driveTr	.wheel.radius*driveTr.motor.RPMmax*60/(1000*driveTr.gearRatio); //km/h    
v.wind=wind;
v.brakeDrag=roadF;  
return v;
}
