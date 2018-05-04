/*
 * motor_util.h
 *
 *  Created on: May 4, 2018
 *      Author: Sheldon
 */

#ifndef MOTOR_UTIL_H_
#define MOTOR_UTIL_H_

#include "libsc/dir_motor.h"
#include "libutil/misc.h"
using namespace libsc;

extern DirMotor* L_motor;
extern DirMotor* R_motor;


void SetPower(int speed,int id){
	int power = (speed > 0?power:-power);
	bool direction = (power > 0);
	power = libutil::Clamp<int>(0,power,1000);
	switch(id){
	case 0:
		L_motor->SetPower(power);
		L_motor->SetClockwise(direction);
		break;
	case 1:
		R_motor->SetPower(power);
		R_motor->SetClockwise(!direction);
	}
}

int GetMotorPower(int id){
	int power;
	switch(id){
	case 0:
		power = L_motor->GetPower();
		return (L_motor->IsClockwise() ? power : -power);//true is forward
		break;
	case 1:
		power = R_motor->GetPower();
		return (R_motor->IsClockwise() ? -power : power);//false is forward
		break;
	}
	return 0;
}
#endif /* MOTOR_UTIL_H_ */
