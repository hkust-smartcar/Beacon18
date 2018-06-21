/*
 * var.h
 *
 *  Created on: Jun 11, 2018
 *      Author: Sheldon
 */

#ifndef INC_VAR_H_
#define INC_VAR_H_

#include "beacon.h"
#include <stdlib.h>
#include "libsc/st7735r.h"
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include "libsc/lcd_typewriter.h"
#include <libsc/k60/ov7725.h>
#include <libsc/dir_encoder.h>
#include "libsc/alternate_motor.h"
#include "libutil/incremental_pid_controller.h"
#include "pid.h"
#include "libbase/k60/pit.h"
#include "libutil/misc.h"

using libsc::System;
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;

struct BeaconPackage {
	Beacon* target = NULL;
	uint16_t received_time = 0;
};

enum PkgType {
	irTarget = 0, oTarget = 1
};
struct BitConsts {
	uint8_t kSTART = 0xF0;
	uint8_t kEND = 0xFF;
};

enum rotate_state {
	no, prepare, performing
};
enum state_ {
	forward, chase, rotation, out, keep,avoid
};

state_ action = forward;
bool run = false;
const Byte* buf = NULL;
Beacon* ir_target = NULL;
Beacon* ir_record = NULL;
BeaconPackage o_target;
BeaconPackage ir_target2;

const uint16_t width = 320;
const uint16_t height = 240;
const uint16_t numOfPixel = 9600;
uint8_t contrast = 0x40;
uint8_t brightness = 0x00;
Led* led0 = NULL;
Led* led1 = NULL;
St7735r* lcd = NULL;
LcdTypewriter* writer = NULL;
Ov7725* cam = NULL;
JyMcuBt106* bt = NULL;
JyMcuBt106* comm = NULL;
AlternateMotor *L_motor = NULL;
AlternateMotor *R_motor = NULL;
DirEncoder* encoder1 = NULL;
DirEncoder* encoder2 = NULL;
BatteryMeter* bMeter = NULL;
IncrementalPidController<int, int>* L_pid = NULL;
IncrementalPidController<int, int>* R_pid = NULL;
PID* Dir_pid = NULL;
Pit* pit = NULL;

void SetPower(int speed, int id) {
	bool direction = (speed > 0);
	int power = (speed > 0 ? speed : -speed);
	power = libutil::Clamp<int>(0, power, 600);
	switch (id) {
	case 0:
		L_motor->SetPower(power);
		L_motor->SetClockwise(direction);
		break;
	case 1:
		R_motor->SetPower(power);
		R_motor->SetClockwise(!direction);
	}
}

int GetMotorPower(int id) {
	switch (id) {
	case 0:
		return L_motor->IsClockwise() ?
				L_motor->GetPower() : -L_motor->GetPower(); //true is forward
		break;
	case 1:
		return R_motor->IsClockwise() ?
				-R_motor->GetPower() : R_motor->GetPower(); //false is forward
		break;
	}
	return 0;
}

std::list<uint8_t> buffer;

inline void BuildBufferPackage() {
	auto it = buffer.begin();
	uint8_t type = *(it++);
	uint8_t x = *(it++);
	uint8_t y = *(it++);
	BeaconPackage* ptr = NULL;
	switch (type) {
	case PkgType::irTarget:
		ptr = &ir_target2;
		break;
	case PkgType::oTarget:
		ptr = &o_target;
		break;
	}
	if (ptr->target == NULL)
		ptr->target = new Beacon();
	ptr->target->center.first = x;
	ptr->target->center.second = y;
	ptr->received_time = System::Time();

}

// void pid_cycle(Pit*){
// 		encoder1->Update();
// 		encoder2->Update();
// 		SetPower(GetMotorPower(0) + L_pid->Calc(encoder1->GetCount()), 0);
// 		SetPower(GetMotorPower(1) + R_pid->Calc(-encoder2->GetCount()), 1);
// }

bool comm_listener(const Byte *data, const size_t size) {
	BitConsts a;
	if (data[0] == a.kSTART)
		buffer.clear();
	else if (data[0] == a.kEND)
		BuildBufferPackage();
	else
		buffer.push_back(data[0]);
	return true;
}

bool bt_listener(const Byte *data, const size_t size) {
	if (data[0] == 's') {
		run = true;
		led0->SetEnable(1);
		comm->SendStrLiteral("s");
		L_pid->SetSetpoint(30);
		R_pid->SetSetpoint(30);
		// pit->SetEnable(true);
	}
	if (data[0] == 'S') {
		run = false;
		led0->SetEnable(0);
		L_motor->SetPower(0);
		R_motor->SetPower(0);
		// pit->SetEnable(false);
		comm->SendStrLiteral("S");
	}
	return true;
}

#endif /* INC_VAR_H_ */
