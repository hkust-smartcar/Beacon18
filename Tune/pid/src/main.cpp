/*
 * main.cpp
 *
 * Author:Sheldon
 * Copyright (c) 2017-2018 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <string>
#include <stdlib.h>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/dir_encoder.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
//#include "libsc/joystick.h"
#include "libsc/st7735r.h"
#include "libsc/battery_meter.h"
#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include <libsc/k60/ov7725.h>
#include "libsc/alternate_motor.h"
#include "motor_util.h"
#include <list>
#include "pid.h"

namespace libbase {
namespace k60 {

Mcg::Config Mcg::GetMcgConfig() {
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}
}
}

using libsc::System;
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;

struct BitConsts {
	uint8_t kSTART = 0xF0;
	uint8_t kEND = 0xFF;
};

float L_kp = 1;
float L_ki = 0.01;
float L_kd = 1;
float R_kp = 1;
float R_ki = 0.01;
float R_kd = 1;

bool move[4] = { }; //up,down,left,right

AlternateMotor* L_motor = NULL;
AlternateMotor* R_motor = NULL;
PID* L_pid = NULL;
PID* R_pid = NULL;
LcdTypewriter* writer = NULL;
St7735r* lcd = NULL;
std::list<uint8_t> buffer;
uint32_t recorded_time = 0;
JyMcuBt106* bt = NULL;

void sendInt(int i) {
	Byte out[5];
	out[0] = i < 0 ? 1 : 0;
	i = abs(i);
	out[1] = (i >> 24) & 0xFF;
	out[2] = (i >> 16) & 0xFF;
	out[3] = (i >> 8) & 0xFF;
	out[4] = i & 0xFF;
	bt->SendBuffer(out, 5);
}

inline void BuildBufferPackage() {
	auto it = buffer.begin();
	Byte temp[4];
	float val;
	int val2;
	auto ptr = L_pid;
	for (int z = 0; z < 2; z++) {
		for (int i = 0; i < 4; i++)
			temp[i] = *(it++);
		memcpy(&val, temp, sizeof(float));
		ptr->kP = val;
		for (int i = 0; i < 4; i++)
			temp[i] = *(it++);
		memcpy(&val, temp, sizeof(float));
		ptr->kI = val;
		for (int i = 0; i < 4; i++)
			temp[i] = *(it++);
		memcpy(&val, temp, sizeof(float));
		ptr->kD = val;
		ptr = R_pid;
	}
	char data[20] = { };
	int y = 15;
	ptr = L_pid;
	for (int z = 0; z < 2; z++) {
		lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
		y += 15;
		sprintf(data, "L_kp: %.4f", ptr->kP);
		writer->WriteBuffer(data, 20);
		lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
		y += 15;
		sprintf(data, "L_ki: %.4f", ptr->kI);
		writer->WriteBuffer(data, 20);
		lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
		y += 15;
		sprintf(data, "L_kd: %.4f", ptr->kD);
		writer->WriteBuffer(data, 20);
		ptr = R_pid;
	}
	bool sign = *(it++);
	for (int i = 0; i < 4; i++)
		temp[i] = *(it++);
	memcpy(&val2, temp, sizeof(int));
	if (sign)
		val2 = -val2;
	L_pid->settarget(val2);
	L_pid->settarget(val2);
	lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
	y += 15;
	sprintf(data, "L_T: %d", L_pid->getTarget());
	writer->WriteBuffer(data, 20);
	sign = *(it++);
	for (int i = 0; i < 4; i++)
		temp[i] = *(it++);
	memcpy(&val2, temp, sizeof(int));
	if (sign)
		val2 = -val2;
	R_pid->settarget(val2);
	R_pid->settarget(val2);
	lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
	y += 15;
	sprintf(data, "R_T: %d", R_pid->getTarget());
	writer->WriteBuffer(data, 20);
}

int main() {
	System::Init();
	//led init
	Led::Config led_config;
	led_config.is_active_low = true;
	led_config.id = 0;
	Led led0(led_config);
	led_config.id = 1;
	Led led1(led_config);

	BatteryMeter::Config bConfig;
	bConfig.voltage_ratio = 0.4;
	BatteryMeter bMeter(bConfig);

	St7735r::Config lcd_config;
	lcd_config.orientation = 0;
	lcd_config.fps = 20;
	St7735r lcd_(lcd_config);
	lcd = &lcd_;

	LcdTypewriter::Config writer_config;
	writer_config.lcd = lcd;
	LcdTypewriter writer_(writer_config);
	writer = &writer_;

	uint32_t tick = System::Time();
	bool g_start = false;

	AlternateMotor::Config motor_config;
	motor_config.id = 0;
	AlternateMotor Lmotor(motor_config);
	L_motor = &Lmotor;
	motor_config.id = 1;
	AlternateMotor Rmotor(motor_config);
	R_motor = &Rmotor;
	//////////////////Encoder init//////////////////
	DirEncoder::Config encoder_config;
	encoder_config.id = 1;
	DirEncoder encoder1(encoder_config);
	encoder_config.id = 0;
	DirEncoder encoder2(encoder_config);

	PID L_pid_(L_kp, L_ki, L_kd);
	L_pid = &L_pid_;
	L_pid->errorSumBound = 10000;
	PID R_pid_(R_kp, R_ki, R_kd);
	R_pid = &R_pid_;
	R_pid->errorSumBound = 10000;

	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 0;
	JyMcuBt106 bt_(config);
	bt = &bt_;
	bool receiving = false;
	bool move_re = false;
	bool stop_re = false;
	bool pid = false;
	bt->SetRxIsr(
			[&move_re,&stop_re,&writer,&led0,&g_start,&led1,&receiving,&pid](const Byte *data, const size_t size) {
				BitConsts a;
				if(move_re) {
					move[ data[0] - '0'] = true;
					pid = false;
					move_re = false;
					return true;
				}
				if(stop_re) {
					move[data[0] - '0'] = false;
					stop_re = false;
					return true;
				}
				if(receiving) {
					if (data[0] == a.kEND) {
						BuildBufferPackage();
						receiving = false;
						pid = true;
					}
					else
					buffer.push_back(data[0]);
					return true;
				}
				if (data[0] == a.kSTART) {
					buffer.clear();
					receiving =true;
					return true;
				}
				if(data[0] == 'm')
				move_re = true;
				if(data[0] == 'p')
				stop_re = true;

				if(data[0] =='s') {
					led0.SetEnable(1);
					g_start = true;
					return true;
				}
				if(data[0]=='S') {
					g_start = false;
					led0.SetEnable(0);
					L_motor->SetPower(0);
					R_motor->SetPower(0);
					L_pid->reset();
					R_pid->reset();
					return true;
				}

				return true;
			});

	lcd->SetRegion(Lcd::Rect(0, 0, 128, 160));
	lcd->Clear(Lcd::kWhite);
	int L_count = 0;
	int R_count = 0;
	char v[10] = { };
	sprintf(v, "v: %f", bMeter.GetVoltage());
	lcd->SetRegion(Lcd::Rect(0, 0, 128, 160));
	writer->WriteBuffer(v, 10);
	uint32_t pid_time = System::Time();

	while (1) {
		if (tick != System::Time() && g_start) {
			tick = System::Time();
			if (tick % 15 == 0) {
				BitConsts a;
				bt->SendBuffer(&a.kSTART, 1);
				sendInt(L_count);
				sendInt(-R_count);
				bt->SendBuffer(&a.kEND, 1);
			}
			if (tick % 11 == 0) {
				if (!(move[0] || move[1] || move[2] || move[3]) && !pid) {
					L_pid->settarget(0);
					R_pid->settarget(0);
				} else if (move[0]) {	//forward
					if (move[2]) {
						L_pid->settarget(100);
						R_pid->settarget(150);
					} else if (move[3]) {
						L_pid->settarget(150);
						R_pid->settarget(100);
					} else {
						L_pid->settarget(100);
						R_pid->settarget(100);
					}
				} else if (move[1]) {	//backward
					if (move[2]) {
						L_pid->settarget(-100);
						R_pid->settarget(-150);
					} else if (move[3]) {
						L_pid->settarget(-150);
						R_pid->settarget(-100);
					} else {
						L_pid->settarget(-100);
						R_pid->settarget(-100);
					}
				} else if (move[2]) {
					L_pid->settarget(-100);
					R_pid->settarget(100);
				} else if (move[3]) {
					L_pid->settarget(100);
					R_pid->settarget(-100);
				}
			}
			if (tick % 10 == 0) {
				uint32_t time_diff = tick - pid_time;
				encoder1.Update();
				encoder2.Update();
				L_count = encoder1.GetCount() * 50;
				R_count = encoder2.GetCount() * 50;
				L_count /= (int) time_diff;
				R_count /= (int) time_diff;
				SetPower(L_pid->output(L_count), 0);
				SetPower(R_pid->output(-R_count), 1);
				pid_time = System::Time();
			}
		}
	}
	return 0;
}
