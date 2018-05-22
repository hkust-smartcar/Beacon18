///*
// * main.cpp
// *
// * Author:Sheldon
// * Copyright (c) 2017-2018 HKUST SmartCar Team
// * Refer to LICENSE for details
// */
//
//#include <cassert>
//#include <cstring>
//#include <stdlib.h>
//#include <string>
//#include <libbase/k60/mcg.h>
//#include <libsc/system.h>
//#include <libsc/dir_encoder.h>
//#include <libsc/led.h>
//#include <libsc/k60/jy_mcu_bt_106.h>
////#include "libsc/joystick.h"
//#include "libsc/st7735r.h"
//#include "libsc/battery_meter.h"
////#include "libbase/k60/pit.h"
//#include "libsc/lcd_typewriter.h"
//#include <libsc/k60/ov7725.h>
//#include "libsc/alternate_motor.h"
//#include "beacon.h"
//#include "libbase/misc_utils_c.h"
//#include "libutil\incremental_pid_controller.h"
//#include "pid.h"
//#include "image_processing.h"
//#include "motor_util.h"
//#include "libutil/misc.h"
//#include <list>
//
//namespace libbase {
//namespace k60 {
//
//Mcg::Config Mcg::GetMcgConfig() {
//	Mcg::Config config;
//	config.external_oscillator_khz = 50000;
//	config.core_clock_khz = 150000;
//	return config;
//}
//}
//}
//
//using libsc::System;
//using namespace libsc;
//using namespace libsc::k60;
//using namespace libbase::k60;
//using namespace libutil;
//
////////////////cam setting////////////////
//const uint16_t width = 320;
//const uint16_t height = 240;
//const uint16_t numOfPixel = width * height / 8;
//uint8_t contrast = 0x40;
//uint8_t brightness = 0x00;
///////////////////PID//////////////////////
//float L_kp = 2.5;
//float L_ki = 0.02;
//float L_kd = 0;
//float R_kp = 2.5;
//float R_ki = 0.02;
//float R_kd = 0;
//float Dir_kp = 0.5;
//float Dir_ki = 0.1;
//float Dir_kd = 0.05;
////////////////algo parm///////////////////
//const uint8_t max_beacon = 10;
//const uint8_t frame = 10;
//const uint16_t near_area = 3000;
//const float target_slope = 0.004927548087104732;
//const float target_intercept = 173.5123409387965;
///////////////state//////////////////////
//Beacon* target = NULL;
//Beacon last_beacon;
//////////////speed//////////////////////
//const int forward_speed = 100;
//const int finding_speed = 70;
//const int rotate_speed = 120;
//
//enum rotate_state {
//	no, prepare, performing
//};
//
//AlternateMotor* L_motor = NULL;
//AlternateMotor* R_motor = NULL;
//
//int main() {
//	System::Init();
//
//	/////////////////////led init////////////////////
//	Led::Config led_config;
//	led_config.is_active_low = true;
//	led_config.id = 0;
//	Led led0(led_config);
//	led_config.id = 1;
//	Led led1(led_config);
//
//	BatteryMeter::Config bConfig;
//	bConfig.voltage_ratio = 0.4;
//	BatteryMeter bMeter(bConfig);
//	/////////////////////LCD init///////////////////
//	St7735r::Config lcd_config;
//	lcd_config.orientation = 1;
//	lcd_config.fps = 60;
//	St7735r lcd(lcd_config);
//	LcdTypewriter::Config writer_config;
//	writer_config.lcd = &lcd;
//	LcdTypewriter writer(writer_config);
//	lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
//	lcd.Clear(Lcd::kWhite);
//	///////////////////Motor init///////////////////
//	AlternateMotor::Config motor_config;
//	motor_config.id = 0;
//	AlternateMotor motor0(motor_config);
//	L_motor = &motor0;
//	motor_config.id = 1;
//	AlternateMotor motor1(motor_config);
//	R_motor = &motor1;
//	//////////////////Encoder init//////////////////
//	DirEncoder::Config encoder_config;
//	encoder_config.id = 1;
//	DirEncoder encoder1(encoder_config);
//	encoder_config.id = 0;
//	DirEncoder encoder2(encoder_config);
//	////////////////////cam init//////////////////
//	Ov7725::Config cam_config;
//	cam_config.id = 0;
//	cam_config.w = width;
//	cam_config.h = height;
//	cam_config.contrast = contrast;
//	cam_config.brightness = brightness;
//	cam_config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
//	k60::Ov7725 cam(cam_config);
//	cam.Start();
//	//////////////////PID init////////////////////
//
//	IncrementalPidController<int, int> L_pid(0, L_kp, L_ki, L_kd);
//	L_pid.SetILimit(0);
//	L_pid.SetOutputBound(-600, 600);
//	IncrementalPidController<int, int> R_pid(0, R_kp, R_ki, R_kd);
//	R_pid.SetILimit(0);
//	R_pid.SetOutputBound(-600, 600);
//	PID Dir_pid(Dir_kp, Dir_ki, Dir_kp);
//	Dir_pid.errorSumBound = 10000;
//	////////////////Variable init/////////////////
//	uint32_t tick = System::Time();
//	uint32_t start;
//	uint32_t end;
//	int L_count = 0;
//	int R_count = 0;
//	uint32_t timer = 0;
//	bool seen = false;
//	int finding_time = 0;
//	rotate_state rotate = no;
//	std::list<Beacon> center_record;
//	uint16_t target_x = 173;
//	uint32_t max_area = 0;
//	Beacon b[max_beacon];
//	Beacon *beacons = b;
//	bool calibrated = false;
//	/////////////////For Dubug////////////////////
//	bool sent = true;
//	uint8_t state = 7;
//	bool run = false;
//	bool restart = false;
//	char avoid_state = 'F';
//	JyMcuBt106::Config config;
//	config.id = 1;
//	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//	JyMcuBt106 bt(config);
//	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k4800;
//	config.id = 2;
//	JyMcuBt106 comm(config);
//	comm.SetRxIsr(
//			[&avoid_state,&L_pid,&R_pid](const Byte *data, const size_t size) {
//				switch(data[0]) {
//					case 'L':
//					L_pid.SetSetpoint(50);
//					R_pid.SetSetpoint(150);
//					avoid_state = 'L';
//					break;
//					case 'R':
//					L_pid.SetSetpoint(150);
//					R_pid.SetSetpoint(50);
//					avoid_state = 'R';
//					break;
//					case 'F':
//					avoid_state = 'F';
//				}
//				return true;
//			});
//	bt.SetRxIsr(
//			[&comm,&L_pid,&R_pid,&led0,&run,&Dir_pid,&restart,&encoder1,&encoder2](const Byte *data, const size_t size) {
//				if(data[0] == 's') {
//					run = true;
//					led0.SetEnable(1);
//					encoder1.Update();
//					encoder2.Update();
//					comm.SendStrLiteral("s");
//				}
//				if(data[0] == 'S') {
//					run = false;
//					led0.SetEnable(0);
//					L_motor->SetPower(0);
//					R_motor->SetPower(0);
//					comm.SendStrLiteral("s");
//				}
//				return true;
//			});
//
//////////////////Main loop////////////////////////
//	while (1) {
//		if (tick != System::Time() && run) {
//			tick = System::Time();
//			////////////////////Debug///////////////////
//			if (tick % 5 == 0) {
//				if (!sent) {
//					char data[10];
//					sprintf(data, "S:%d\n", state);
//					bt.SendStr(data);
//					sent = true;
//				}
//			}
//			////////////////////////////////////////////
//			////////////////////PID///////////////////////
//			if (tick % 10 == 0) {
//				encoder1.Update();
//				L_count = encoder1.GetCount();
//				encoder2.Update();
//				R_count = encoder2.GetCount();
//				SetPower(GetMotorPower(0) + L_pid.Calc(L_count), 0);
//				SetPower(GetMotorPower(1) + R_pid.Calc(-R_count), 1);
//			}
//			if (tick % 15 == 0 && avoid_state == 'F') {
//				///////////////decision making///////////////
//				const Byte* buf = cam.LockBuffer();
//				////////////init value///////////////////////
//				uint8_t beacon_count = 0;
//				target = NULL;
//				///////////////process image/////////////////
//				process(buf, beacons, beacon_count, seen, center_record);
//				if (target != NULL) {		//target find
//					char data[20];
//					sprintf(data, "I:%d,%d\n", target_x,
//							target->area);
//					bt.SendStr(data);
//					if (target->area > max_area)
//						max_area = target->area;
//					if (target->area > near_area && rotate == no)
//						rotate = prepare;
//					if (!calibrated
//							|| (rotate == performing
//									&& target->center.first > target_x)) {
//					} else {
//						if (rotate == performing)
//							rotate = no;
//						int diff = Dir_pid.output(target_x,
//								target->center.first);
//						if (diff > 0) {
//							if (state != 0) {
//								state = 0;
//								sent = false;
//							}
//							L_pid.SetSetpoint(forward_speed - diff);
//							R_pid.SetSetpoint(forward_speed + diff);
//						} else {
//							if (state != 1) {
//								state = 1;
//								sent = false;
//							}
//							L_pid.SetSetpoint(forward_speed + abs(diff));
//							R_pid.SetSetpoint(forward_speed + diff);
//						}
//					}
//					last_beacon.area = target->area;
//					last_beacon.center = target->center;
//					if (!seen)
//						seen = true;
//					if (start)
//						start = 0;
//				} else if (rotate == performing) {//target not find and rotating
//					if (System::Time() - timer > 800) {
//						L_pid.SetSetpoint(-finding_speed);
//						R_pid.SetSetpoint(forward_speed);
//					}
//
//				} else if (seen) { //target not find but have seen target before
//					if (!calibrated)
//						calibrated = true;
//					if (start == 0){
//						target_x = target_slope * max_area + target_intercept;
////						max_area = 0;
//						start = System::Time();
//					}
//					else if (tick - start > 75) { //target lost for more than 75 ms
//						if (rotate == prepare) {	//done with the last target
//							rotate = performing;
//							L_pid.SetSetpoint(rotate_speed);
//							R_pid.SetSetpoint(rotate_speed);
//							max_area = 0;
//							if (state != 2) {
//								state = 2;
//								sent = false;
//							}
//						}
//						seen = false;
//						calibrated = false;
//						start = 0;
//						Dir_pid.reset();
//						timer = System::Time();
//						if (state != 3 && state != 2) {
//							state = 3;
//							sent = false;
//						}
//					}
//				} else { //target not find and have not seen target before
//					if (finding_time == 0)
//						finding_time = System::Time();
//					else if (tick - finding_time > 1000) {//change to rotate after going foward for 1000ms
//						L_pid.SetSetpoint(finding_speed);
//						R_pid.SetSetpoint(-finding_speed);
//					} else {
//						L_pid.SetSetpoint(finding_speed);
//						R_pid.SetSetpoint(finding_speed);
//					}
//					if (state != 4) {
//						state = 4;
//						sent = false;
//					}
//				}
//
//				cam.UnlockBuffer();
//			}
//		}
//	}
//	return 0;
//}
