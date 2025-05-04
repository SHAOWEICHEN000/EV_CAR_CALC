#ifndef DEFINE_H
#define DEFINE_H
#define pi	       	   (3.1415926)
//---------------battery parameter-----------
#define CELL_CAPACITY  	   (15.0)
#define CELL_WEIGHT    	   (450.0)
#define CELL_VMAX      	   (4.2)
#define CELL_VNOM      	   (3.8)
#define CELL_VMIN      	   (3.0)
#define MODULE_P       	   (3.0)
#define MODULE_S       	   (8.0)
#define MODULE_OVHD    	   (0.08)
#define PACK_S	       	   (12.0)
#define PACK_OVHD      	   (0.1)
#define PACK_SOCF      	   (100.0)  //from 100%
#define PACK_SOCE      	   (25.0)   //to 25%
#define PACK_ETA       	   (0.96)

//----------------mechanic parameter-------------------------
#define WHEEL_RADIUS   	   (0.31)
#define WHEEL_INERTIA  	   (0.6)
#define WHEEL_ROLL  	   (0.012)
#define MOTOR_LMAX     	   (300.0)
#define MOTOR_RPMR     	   (4000.0)
#define MOTOR_RPMMAX   	   (12000.0)
#define MOTOR_ETA          (0.92)
#define MOTOR_INERTIA      (0.2)
#define DRIVE_TRAIN_INVEFF (0.95)
#define DRIVE_TRAIN_REGEN  (0.4)
#define DRIVE_TRAIN_RATIO  (9.1)
#define DRIVE_TRAIN_GEARJ  (0.08)
#define DRIVE_TRAINGEAREFF (0.97)


#endif
