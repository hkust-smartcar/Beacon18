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
#include "libutil\incremental_pid_controller.h"
#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include <libsc/k60/ov7725.h>
#include "libsc/alternate_motor.h"
#include "motor_util.h"
#include <list>

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

float L_kp = 0;
float L_ki = 0;
float L_kd = 0;
float R_kp = 0;
float R_ki = 0;
float R_kd = 0;

AlternateMotor* L_motor = NULL;
AlternateMotor* R_motor = NULL;
IncrementalPidController<int, int>* L_pid = NULL;
IncrementalPidController<int, int>* R_pid = NULL;
LcdTypewriter* writer = NULL;
St7735r* lcd = NULL;
std::list<uint8_t> buffer;

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
		ptr->SetKp(val);
		for (int i = 0; i < 4; i++)
			temp[i] = *(it++);
		memcpy(&val, temp, sizeof(float));
		ptr->SetKi(val);
		for (int i = 0; i < 4; i++)
			temp[i] = *(it++);
		memcpy(&val, temp, sizeof(float));
		ptr->SetKd(val);
		ptr = R_pid;
	}
	char data[20] = { };
	int y = 15;
	ptr = L_pid;
	for (int z = 0; z < 2; z++) {
		lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
		y += 15;
		sprintf(data, "L_kp: %.4f", ptr->GetKp());
		writer->WriteBuffer(data, 20);
		lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
		y += 15;
		sprintf(data, "L_ki: %.4f", ptr->GetKi());
		writer->WriteBuffer(data, 20);
		lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
		y += 15;
		sprintf(data, "L_kd: %.4f", ptr->GetKd());
		writer->WriteBuffer(data, 20);
		ptr = R_pid;
	}
	bool sign = *(it++);
	for (int i = 0; i < 4; i++)
		temp[i] = *(it++);
	memcpy(&val2, temp, sizeof(int));
	if (sign)
		val2 = -val2;
	L_pid->SetSetpoint(val2);
	lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
	y += 15;
	sprintf(data, "L_T: %d", L_pid->GetSetpoint());
	writer->WriteBuffer(data, 20);
	sign = *(it++);
	for (int i = 0; i < 4; i++)
		temp[i] = *(it++);
	memcpy(&val2, temp, sizeof(int));
	if (sign)
		val2 = -val2;
	R_pid->SetSetpoint(val2);
	lcd->SetRegion(Lcd::Rect(0, y, 128, 15));
	y += 15;
	sprintf(data, "R_T: %d", R_pid->GetSetpoint());
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

	IncrementalPidController<int, int> L_pid_(0, L_kp, L_ki, L_kd);
	L_pid = &L_pid_;
	L_pid->SetILimit(0);
	L_pid->SetOutputBound(-1000, 1000);
	IncrementalPidController<int, int> R_pid_(0, R_kp, R_ki, R_kd);
	R_pid = &R_pid_;
	R_pid->SetILimit(0);
	R_pid->SetOutputBound(-1000, 1000);

	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 0;
	JyMcuBt106 bt(config);
	bool receiving = false;
	bt.SetRxIsr(
			[&encoder1,&encoder2,&writer,&led0,&g_start,&led1,&bt,&receiving](const Byte *data, const size_t size) {
				BitConsts a;
				if(receiving) {
					if (data[0] == a.kEND) {
						BuildBufferPackage();
						receiving = false;
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
				if(data[0] =='s') {
					led0.SetEnable(1);
					g_start = true;
					encoder1.Update();
					encoder2.Update();
					return true;
				}
				if(data[0]=='S') {
					g_start = false;
					led0.SetEnable(0);
					L_pid->SetSetpoint(0);
					R_pid->SetSetpoint(0);
					L_motor->SetPower(0);
					R_motor->SetPower(0);
					L_pid->Reset();
					R_pid->Reset();
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
		if (tick != System::Time()) {
			tick = System::Time();
			if (tick % 30 == 0 && g_start) {
				Byte out[12];
				BitConsts a;
				int temp = abs(L_count);
				out[0] = a.kSTART;
				out[1] = L_count < 0 ? 1 : 0;
				out[2] = (temp >> 24) & 0xFF;
				out[3] = (temp >> 16) & 0xFF;
				out[4] = (temp >> 8) & 0xFF;
				out[5] = temp & 0xFF;
				temp = abs(R_count);
				out[6] = R_count < 0 ? 1 : 0;
				out[7] = (temp >> 24) & 0xFF;
				out[8] = (temp >> 16) & 0xFF;
				out[9] = (temp >> 8) & 0xFF;
				out[10] = temp & 0xFF;
				out[11] = a.kEND;
				bt.SendBuffer(out, 12);
			}
			if (tick % 10 == 0 && g_start) {
				uint32_t time_diff = tick - pid_time;
				encoder1.Update();
				encoder2.Update();
				L_count = encoder1.GetCount();
				R_count = encoder2.GetCount();
				int32_t reading1 = L_count * 10;
				int32_t reading2 = R_count * 10;
				reading1 = reading1 / (int) time_diff;
				reading2 = reading2 / (int) time_diff;
//				SetPower(GetMotorPower(0) + L_pid.Calc(reading1), 0);
//				SetPower(GetMotorPower(1) + R_pid.Calc(-reading2), 1);
				pid_time = System::Time();
			}
		}
	}
	return 0;
}
