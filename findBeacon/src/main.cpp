/*
 * main.cpp
 *
 * Author:Sheldon
 * Copyright (c) 2017-2018 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <stdlib.h>
#include <string>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/dir_encoder.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
//#include "libsc/joystick.h"
#include "libsc/st7735r.h"
//#include "libsc/battery_meter.h"
//#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include <libsc/k60/ov7725.h>
#include "libsc/dir_motor.h"
#include "beacon.h"
#include "libbase/misc_utils_c.h"
#include "pid.h"
#include "image_processing.h"
#include "motor_util.h"

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

//////////////cam setting////////////////
const uint16_t width = 80;
const uint16_t height = 60;
const uint16_t numOfPixel = width * height / 8;
uint8_t contrast = 0x40;
uint8_t brightness = 0x00;
/////////////////PID//////////////////////
float L_kp = 0.1;
float L_ki = 0.12;
float L_kd = 0.01;
float R_kp = 0.1;
float R_ki = 0.12;
float R_kd = 0.01;
float Dir_kp = 0.2;
float Dir_ki = 0.1;
float Dir_kd = 0.05;
//////////////algo parm///////////////////
const uint8_t max_beacon = 10;
const uint16_t near_area = 3000;
/////////////state//////////////////////
Beacon* target = NULL;
Beacon last_beacon;
////////////speed//////////////////////
const int forward_speed = 120;
const int finding_speed = 100;
const int rotate_speed = 200;

enum rotate_state {
	no, prepare, performing
};

DirMotor* L_motor = NULL;
DirMotor* R_motor = NULL;

int main() {
	System::Init();

	/////////////////////led init////////////////////
	Led::Config led_config;
	led_config.is_active_low = true;
	led_config.id = 0;
	Led led0(led_config);
	led_config.id = 1;
	Led led1(led_config);
	/////////////////////LCD init///////////////////
	St7735r::Config lcd_config;
	lcd_config.orientation = 1;
	lcd_config.fps = 60;
	St7735r lcd(lcd_config);
	LcdTypewriter::Config writer_config;
	writer_config.lcd = &lcd;
	LcdTypewriter writer(writer_config);
	lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd.Clear(Lcd::kWhite);
	///////////////////Motor init///////////////////
	DirMotor::Config motor_config;
	motor_config.id = 0;
	DirMotor motor0(motor_config);
	L_motor = &motor0;
	motor_config.id = 1;
	DirMotor motor1(motor_config);
	R_motor = &motor1;
	//////////////////Encoder init//////////////////
	DirEncoder::Config encoder_config;
	encoder_config.id = 1;
	DirEncoder encoder1(encoder_config);
	encoder_config.id = 0;
	DirEncoder encoder2(encoder_config);
	////////////////////cam init//////////////////
	Ov7725::Config cam_config;
	cam_config.id = 0;
	cam_config.w = width;
	cam_config.h = height;
	cam_config.contrast = contrast;
	cam_config.brightness = brightness;
	cam_config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 cam(cam_config);
	cam.Start();
	//////////////////PID init////////////////////
	PID L_pid(L_kp, L_ki, L_kd);
	L_pid.errorSumBound = 10000;
	PID R_pid(R_kp, R_ki, R_kd);
	R_pid.errorSumBound = 10000;
	PID Dir_pid(Dir_kp, Dir_ki, Dir_kp);
	Dir_pid.errorSumBound = 10000;
	////////////////Variable init/////////////////
	uint32_t tick = System::Time();
	uint32_t start;
	uint32_t end;
	int L_count = 0;
	int R_count = 0;
	int L_speed = 0;
	int R_speed = 0;
	int L_target_count = 0;
	int R_target_count = 0;
	uint8_t frame_count = 0;
	Beacon center_record[10];
	uint32_t timer = 0;
	bool seen = false;
	rotate_state rotate = no;
	/////////////////For Dubug////////////////////
	bool sent = true;
	uint8_t state = 7;
	bool run = false;
	bool restart = false;
	bool receiving[3] = { false };
	bool pos_sent = true;
	uint32_t temp_data = 0;
	int8_t factor = 0;
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 1;
	JyMcuBt106 bt(config);
	config.id = 2;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k9600;
	JyMcuBt106 comm(config);
	bool comm_receiving[5] = { false };
	uint16_t comm_temp_data = 0;
	int8_t comm_factor = 0;
	Beacon temp_target;

	comm.SetRxIsr(
			[&lcd,&writer,&comm_receiving,&comm_temp_data,&comm_factor,&led0,&temp_target](const Byte *data, const size_t size) {
				if(data[0] == 'n') {
					target = NULL;
//					led0.Switch();
					return true;
				}
				if(data[0] == 'x') {
					comm_temp_data = 0;
					comm_receiving[0] = true;
					comm_factor = 8;
					return true;
				}
				if(data[0] == 'y') {
					comm_temp_data = 0;
					comm_receiving[1] = true;
					comm_factor = 8;
					return true;
				}
				if(comm_receiving[0] == true) {
					comm_temp_data |= data[0] << comm_factor;
					comm_factor -= 8;
					if(comm_factor < 0 ) {
						char out[20]= {};
						comm_receiving[0] = false;
						temp_target.center.first = comm_temp_data;
						if(comm_temp_data > 320) {
							int temp = 0;
						}
						sprintf(out,"x: %d",temp_target.center.first);
						lcd.SetRegion(Lcd::Rect(0,0,128,15));
						writer.WriteBuffer(out,10);
					}
					return true;
				}

				if(comm_receiving[1] == true) {
					comm_temp_data |= data[0] << comm_factor;
					comm_factor -= 8;
					if(comm_factor < 0 ) {
						char out[20]= {};
						comm_receiving[1] = false;
						temp_target.center.second = comm_temp_data;
						if(comm_temp_data > 320) {
							int temp = 0;
						}
						sprintf(out,"y: %d",temp_target.center.second);
						lcd.SetRegion(Lcd::Rect(0,15,128,15));
						writer.WriteBuffer(out,10);
					}
					return true;
				}

				return false;
			});
//	bt.SetRxIsr(
//			[&lcd,&writer,&L_pid,&R_pid,&led0,&factor,&run,&receiving,&Dir_pid,&temp_data,&restart,&encoder1,&encoder2](const Byte *data, const size_t size) {
//				if(data[0] == 's') {
//					run = true;
//					led0.Switch();
//					encoder1.Update();
//					encoder2.Update();
//				}
//				if(data[0] == 'S') {
//					restart = true;
//					led0.SetEnable(0);
//					L_motor->SetPower(0);
//					R_motor->SetPower(0);
//				}
//				if(receiving[0] == true) {
//					temp_data |= data[0] << factor;
//					factor -= 8;
//					if(factor < 0 ) {
//						receiving[0] = false;
//						memcpy(&L_kp,&temp_data,sizeof(L_kp));
//						char out[20]= {};
//						sprintf(out,"L_kp: %f",L_kp);
//						lcd.SetRegion(Lcd::Rect(0,0,128,15));
//						writer.WriteBuffer(out,10);
//						L_pid.kP = L_kp;
//					}
//				}
//				if(receiving[1] == true) {
//					temp_data |= data[0] << factor;
//					factor -= 8;
//					if(factor < 0 ) {
//						receiving[1] = false;
//						memcpy(&L_ki,&temp_data,sizeof(L_ki));
//						char out[20]= {};
//						sprintf(out,"L_ki: %f",L_ki);
//						lcd.SetRegion(Lcd::Rect(0,32,128,15));
//						writer.WriteBuffer(out,15);
//						L_pid.kI = L_ki;
//					}
//				}
//				if(receiving[2] == true) {
//					temp_data |= data[0] << factor;
//					factor -= 8;
//					if(factor < 0 ) {
//						receiving[2] = false;
//						memcpy(&L_kd,&temp_data,sizeof(L_kd));
//						char out[20]= {};
//						sprintf(out,"L_kd: %f",L_kd);
//						lcd.SetRegion(Lcd::Rect(0,16,128,15));
//						writer.WriteBuffer(out,15);
//						L_pid.kD = L_kd;
//					}
//				}
//				if(receiving[3] == true) {
//					temp_data |= data[0] << factor;
//					factor -= 8;
//					if(factor < 0 ) {
//						receiving[3] = false;
//						memcpy(&R_kp,&temp_data,sizeof(R_kp));
//						char out[20]= {};
//						sprintf(out,"R_kp: %f",R_kp);
//						lcd.SetRegion(Lcd::Rect(0,65,128,15));
//						writer.WriteBuffer(out,15);
//						R_pid.kP = R_kp;
//					}
//				}
//				if(receiving[4] == true) {
//					temp_data |= data[0] << factor;
//					factor -= 8;
//					if(factor < 0 ) {
//						receiving[4] = false;
//						memcpy(&R_ki,&temp_data,sizeof(R_ki));
//						char out[20]= {};
//						sprintf(out,"R_ki: %f",R_ki);
//						lcd.SetRegion(Lcd::Rect(0,80,128,15));
//						writer.WriteBuffer(out,15);
//						R_pid.kI = R_ki;
//					}
//				}
//				if(receiving[5] == true) {
//					temp_data |= data[0] << factor;
//					factor -= 8;
//					if(factor < 0 ) {
//						receiving[5] = false;
//						memcpy(&R_kd,&temp_data,sizeof(R_kd));
//						char out[20]= {};
//						sprintf(out,"R_kd: %f",R_kd);
//						lcd.SetRegion(Lcd::Rect(0,95,128,15));
//						writer.WriteBuffer(out,15);
//						R_pid.kD = R_kd;
//					}
//				}
//
//				if(data[0] == 'p') {
//					temp_data = 0;
//					receiving[0] = true;
//					factor = 24;
//				}
//				if(data[0] == 'i') {
//					temp_data = 0;
//					receiving[1] = true;
//					factor = 24;
//				}
//				if(data[0] == 'd') {
//					temp_data = 0;
//					receiving[2] = true;
//					factor = 24;
//				}
//				if(data[0] == 'P') {
//					temp_data = 0;
//					receiving[3] = true;
//					factor = 24;
//				}
//				if(data[0] == 'I') {
//					temp_data = 0;
//					receiving[4] = true;
//					factor = 24;
//				}
//				if(data[0] == 'D') {
//					temp_data = 0;
//					receiving[5] = true;
//					factor = 24;
//				}
//				return true;
//			});

	////////////////Main loop////////////////////////
	while (1) {
		if (tick != System::Time() && run) {
			if (restart) {
				run = false;
				seen = false;
				start = 0;
				L_target_count = 0;
				R_target_count = 0;
				state = 7;
				sent = true;
				restart = false;
				L_pid.reset();
				R_pid.reset();
				Dir_pid.reset();
				continue;
			}
			tick = System::Time();
//			if (tick % 30 == 0) {
//				////////////////////Debug///////////////////
//				char data[20] = { };
//				sprintf(data, "E:%d,%d\n", L_count, R_count);
//				bt.SendStr(data); /**/
//				sprintf(data, "T:%d,%d\n", L_target_count, R_target_count);
//				bt.SendStr(data);
//				if (!sent) {
//					sprintf(data, "S:%d\n", state);
//					bt.SendStr(data);
//					sent = true;
//				}
//				//				if (!pos_sent) {
//				//					sprintf(data, "P:%d,%d\n", last_beacon.center.first,
//				//							last_beacon.center.second);
//				//					bt.SendStr(data);
//				//					pos_sent = true;
//				//				}
//				////////////////////////////////////////////
//			}
			////////////////////PID///////////////////////
			if (tick % 10 == 0) {
				encoder1.Update();
				L_count = encoder1.GetCount();
				encoder2.Update();
				R_count = encoder2.GetCount();
				/////////////Left motor///////////////////
				L_speed = L_pid.output(L_target_count, L_count);
				SetPower(L_speed, 0);
				////////////Right motor///////////////////
				R_speed = R_pid.output(R_target_count, -R_count);
				SetPower(R_speed, 1);
			}
//			if (tick % 25 == 0) {
//				///////////////decision making///////////////
//				if (target != NULL) {		//target find
//					frame_count = 0;
//					pos_sent = false;
//					if (target->area > near_area && rotate == no)
//						rotate = prepare;
//					if (rotate == performing && target->center.first > 160
//							&& target->center.first < 190) {
//						rotate = no;
//					} else {
//						int diff = Dir_pid.output(200, target->center.first);
//						if (diff > 0) {
//							if (state != 0) {
//								state = 0;
//								sent = false;
//							}
//							R_target_count = forward_speed + diff;
//							L_target_count = forward_speed;
//						} else {
//							if (state != 1) {
//								state = 1;
//								sent = false;
//							}
//							R_target_count = forward_speed;
//							L_target_count = forward_speed + abs(diff);
//						}
//						last_beacon.area = target->area;
//						last_beacon.center = target->center;
//						if (!seen)
//							seen = true;
//						if (start)
//							start = 0;
//					}
//				} else if (rotate == performing) {
//					if (System::Time() - timer > 500) {
//						R_target_count = forward_speed;
//						L_target_count = -finding_speed;
//					}
//
//				} else if (seen) { //target not find but have seen target before
//					if (start == 0)
//						start = System::Time();
//					else if (tick - start > 75) {
//						if (rotate == prepare) {		//went over the target
//							rotate = performing;
//							if (last_beacon.center.first < 170) {//rotate around the beacon
//								R_target_count = rotate_speed;
//								L_target_count = finding_speed;
//								if (state != 2) {
//									state = 2;
//									sent = false;
//								}
//							} else {
//								R_target_count = finding_speed;
//								L_target_count = rotate_speed;
//								if (state != 3) {
//									state = 3;
//									sent = false;
//								}
//							}
//						}
//						seen = false;
//						start = 0;
//						Dir_pid.reset();
//						timer = System::Time();
//						if (state != 4 && state != 3 && state != 2) {
//							state = 4;
//							sent = false;
//						}
//					}
//				} else { //target not find and have not seen target before
//					L_target_count = finding_speed;
//					R_target_count = finding_speed;
//					if (state != 5) {
//						state = 5;
//						sent = false;
//					}
//				}
//				cam.UnlockBuffer();
//
//			}
		}
	}
	return 0;
}
