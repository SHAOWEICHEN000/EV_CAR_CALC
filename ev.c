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
//#include "solar.h"
#include"battery.h"
#include "mechanic.h"
//#include"throttle.h"
//#include "plot_bridge.h"
#include "define.h"
int main() {
    Cell c=setupCell(CELL_CAPACITY,CELL_WEIGHT,CELL_VMAX,CELL_VNOM,CELL_VMIN);
    module_cell m= setupModule(MODULE_P, MODULE_S, MODULE_OVHD, c);
    pack battery=setupPack(PACK_S,PACK_OVHD,PACK_SOCF,PACK_SOCE,PACK_ETA,m);
    wheel wheel=setupWheel(WHEEL_RADIUS,WHEEL_INERTIA,WHEEL_ROLL);
    motor motor=setupMotor(MOTOR_LMAX,MOTOR_RPMR,MOTOR_RPMMAX, MOTOR_ETA,MOTOR_INERTIA);
    dt    driveTr=setupDriveTrain(DRIVE_TRAIN_INVEFF,DRIVE_TRAIN_REGEN,DRIVE_TRAIN_RATIO,DRIVE_TRAIN_GEARJ,DRIVE_TRAINGEAREFF, battery,motor,wheel);
    printf("Drive_train_efficiency=%f",driveTr.efficiency);
    return 0;
}

