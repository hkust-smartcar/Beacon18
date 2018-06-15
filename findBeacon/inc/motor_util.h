/*
 * motor_util.h
 *
 *  Created on: 2018¦~6¤ë15¤é
 *      Author: User
 */

#ifndef INC_MOTOR_UTIL_H_
#define INC_MOTOR_UTIL_H_



#include "var.h"
#include "libutil/misc.h"
using namespace libsc;

void SetPower(int speed,int id){
	bool direction = (speed > 0);
	int power = (speed > 0?speed:-speed);
	power = libutil::Clamp<int>(0,power,600);
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
	switch(id){
	case 0:
		return L_motor->IsClockwise() ? L_motor->GetPower() : -L_motor->GetPower();//true is forward
		break;
	case 1:
		return R_motor->IsClockwise() ? -R_motor->GetPower() : R_motor->GetPower();//false is forward
		break;
	}
	return 0;
}


#endif /* INC_MOTOR_UTIL_H_ */
