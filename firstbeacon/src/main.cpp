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
#include "libsc/alternate_motor.h"
#include "beacon.h"
#include "libbase/misc_utils_c.h"
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

//////////////cam setting////////////////
const uint16_t width = 320;
const uint16_t height = 240;
const uint16_t numOfPixel = width * height / 8;
uint8_t contrast = 0x40;
uint8_t brightness = 0x00;
/////////////////PID//////////////////////
float L_kp = 1;
float L_ki = 0.12;
float L_kd = 0.01;
float R_kp = 1;
float R_ki = 0.12;
float R_kd = 0.01;
float Dir_kp = 2;
float Dir_ki = 0.1;
float Dir_kd = 0.05;
//////////////algo parm///////////////////
const uint8_t x_range = 5;
const uint8_t y_range = 35;
const uint16_t min_size = 120;
const uint8_t error = 10;
const uint8_t max_beacon = 10;
const uint16_t critical_density = 65;
const uint16_t near_area = 3000;
/////////////state//////////////////////
Beacon* target = NULL;
Beacon last_beacon;
bool seen = false;
////////////speed//////////////////////
const int forward_speed = 600;
const int finding_speed = 500;
const int rotate_speed = 1000;

int16_t getX(uint16_t m_pos, int8_t m_bit_pos) {
	int16_t x = (m_pos + 1) * 8 % width - m_bit_pos - 1;
	if (x < 0)
		x = width - m_bit_pos - 1;
	return x;
}

int16_t getY(uint16_t m_pos) {
	int16_t y = m_pos * 8 / width;
	return y;
}

bool check_target(Beacon beacons[max_beacon], uint8_t beacon_count) {
	if (beacons[beacon_count].count > min_size)
		if (beacons[beacon_count].density > critical_density) {
			target = beacons + beacon_count;
			return true;
		}
	return false;
}

bool skip(uint16_t m_pos, int8_t m_bit_pos, Beacon beacons[max_beacon],
		uint8_t beacon_count) {
	if (beacon_count == 0)
		return false;
	int16_t x = getX(m_pos, m_bit_pos);
	int16_t y = getY(m_pos);
	for (uint8_t i = 0; i < beacon_count; i++) {
		if (x < beacons[i].right_x + error && y < beacons[i].lower_y + error
				&& y
						> (beacons[i].upper_y - error < 0 ?
								0 : beacons[i].upper_y - error)
				&& x
						> (beacons[i].left_x - error < 0 ?
								0 : beacons[i].left_x - error)) {
			return true;
		}
	}
	return false;
}

// 1 = upper left, 2 = upper right , 3 = lower left, 4 = lower right
void sub_scan(const Byte* buf, uint16_t m_pos, int8_t m_bit_pos,
		Beacon* current, int dir) {

	switch (dir) {
	case 2:
		if (--m_bit_pos < 0) {
			m_bit_pos = 7;
			++m_pos;
		}
	case 1:
		if (m_pos - width / 8 < 0)
			return;
		else
			m_pos -= width / 8;
		break;
	case 4:
		if (--m_bit_pos < 0) {
			m_bit_pos = 7;
			++m_pos;
		}
	}

	uint8_t x_count = 0;
	uint8_t y_count = 0;
	uint8_t i = 0;
	bool all_black = true;
	int16_t x = 0;
	int16_t y = getY(m_pos);
	int16_t temp_pos = 0;

	while (x_count < x_range) {
		while (y_count < y_range) {
			temp_pos = dir > 2 ? m_pos + width * i / 8 : m_pos - width * i / 8;
			if (temp_pos >= numOfPixel || temp_pos < 0)
				break;
			if (!GET_BIT(buf[temp_pos], m_bit_pos)) {
				int16_t temp_y = dir > 2 ? y + i : y - i;
				x = getX(temp_pos, m_bit_pos);
				all_black = false;
				current->count++;
				if (dir % 2 != 0) {					//scan left
					if (x < current->left_x)
						current->left_x = x;
				} else if (x > current->right_x) 	//scan right
					current->right_x = x;
				if (dir > 2) {						//scan lower
					if (temp_y > current->lower_y)
						current->lower_y = temp_y;
				} else if (temp_y < current->upper_y)	//scan upper
					current->upper_y = temp_y;
				//	y_count = 0;
			} else
				y_count++;
			i++;
		}

		if (all_black)
			x_count++;
		//		else
		//			x_count = 0;
		if (dir % 2 != 0) {				//scan left
			if (++m_bit_pos > 7) {
				m_bit_pos = 0;
				--m_pos;
			}
		} else if (--m_bit_pos < 0) {		//scan right
			m_bit_pos = 7;
			++m_pos;
		}
		y_count = 0;
		i = 0;
		all_black = true;
		if (m_pos * 8 / width != y)
			break;
	}
}

bool scan(const Byte* buf, uint16_t m_pos, int8_t m_bit_pos,
		Beacon beacons[max_beacon], uint8_t &beacon_count, int mode) {

	int16_t x = getX(m_pos, m_bit_pos);
	int16_t y = getY(m_pos);
	beacons[beacon_count].init(x, y);

	//scan lower left
	sub_scan(buf, m_pos, m_bit_pos, beacons + beacon_count, 3);
	//scan lower right
	sub_scan(buf, m_pos, m_bit_pos, beacons + beacon_count, 4);

	if (mode == 1) {
		sub_scan(buf, m_pos, m_bit_pos, beacons + beacon_count, 1);
		sub_scan(buf, m_pos, m_bit_pos, beacons + beacon_count, 2);
	}

	beacons[beacon_count].calc();
	if (check_target(beacons, beacon_count))
		return true;
	else
		beacon_count++;
	return false;
}

bool process(const Byte* buf, Beacon beacons[max_beacon],
		uint8_t &beacon_count) {

	uint16_t pos = 0;
	int8_t bit_pos = 8;
	//////check for beacon with the last recorded pos/////////
	if (seen) {
		uint16_t temp_pos = (width * last_beacon.center.second) / 8
				+ last_beacon.center.first / 8;
		uint16_t temp_bit_pos = 8 - (last_beacon.center.first % 8 + 1);
		scan(buf, temp_pos, temp_bit_pos, beacons, beacon_count, 1);
		if (beacons[0].count > 100) {
			target = beacons;
			return true;
		} else
			beacon_count = 0;
	}
	for (uint16_t y = 0; y < height; y++) {
		for (uint16_t x = 0; x < width; ++x) {
			if (--bit_pos < 0) {
				bit_pos = 7;
				++pos;
			}
			if (!GET_BIT(buf[pos], bit_pos)) {
				if (skip(pos, bit_pos, beacons, beacon_count))
					continue;
				if (scan(buf, pos, bit_pos, beacons, beacon_count, 0))
					return true;
				if (beacon_count == max_beacon)
					return false;
			}
		}
	}
	return false;
}
bool check_near(Beacon b1, Beacon b2) {
	if (abs(b1.center.first - b2.center.first) < 40)
		if (abs(b1.center.second - b2.center.second) < 40)
			return true;
	return false;
}

enum rotate_state {
	no, prepare, performing
};

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
	AlternateMotor::Config motor_config;
	motor_config.id = 0;
	AlternateMotor L_motor(motor_config);
	motor_config.id = 1;
	AlternateMotor R_motor(motor_config);
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
	rotate_state rotate = no;
	/////////////////For Dubug////////////////////
	bool sent = true;
	uint8_t state = 4;
	bool run = false;
	bool restart = false;
	bool receiving[3] = { false };
	bool pos_sent = true;
	uint32_t temp_data = 0;
	int8_t factor = 0;
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 1;
	config.tx_buf_size = 14;
	JyMcuBt106 bt(config);
	bt.SetRxIsr(
			[&led0,&factor,&run,&receiving,&Dir_pid,&temp_data,&restart,&L_motor,&R_motor,&encoder1,&encoder2](const Byte *data, const size_t size) {
				if(data[0] == 's') {
					run = true;
					led0.Switch();
					encoder1.Update();
					encoder2.Update();
				}
				if(data[0] == 'S') {
					restart = true;
					led0.SetEnable(0);
					L_motor.SetPower(0);
					R_motor.SetPower(0);
				}
				if(receiving[0] == true) {
					temp_data |= data[0] << factor;
					factor -= 8;
					if(factor < 0 ) {
						receiving[0] = false;
						memcpy(&Dir_pid.kP,&temp_data,sizeof(Dir_kp));
					}
				}
				if(receiving[1] == true) {
					temp_data |= data[0] << factor;
					factor -= 8;
					if(factor < 0 ) {
						receiving[1] = false;
						memcpy(&Dir_pid.kI,&temp_data,sizeof(Dir_kp));
					}
				}
				if(receiving[2] == true) {
					temp_data |= data[0] << factor;
					factor -= 8;
					if(factor < 0 ) {
						receiving[2] = false;
						memcpy(&Dir_pid.kD,&temp_data,sizeof(Dir_kp));
					}
				}
				if(data[0] == 'p') {
					temp_data = 0;
					receiving[0] = true;
					factor = 24;
				}
				if(data[0] == 'i') {
					temp_data = 0;
					receiving[1] = true;
					factor = 24;
				}
				if(data[0] == 'd') {
					temp_data = 0;
					receiving[2] = true;
					factor = 24;
				}
				return true;
			});

	////////////////Main loop////////////////////////
	while (1) {
		if (tick != System::Time() && run) {
			if (restart) {
				run = false;
				seen = false;
				start = 0;
				L_target_count = 0;
				R_target_count = 0;
				state = 4;
				sent = true;
				restart = false;
				L_pid.reset();
				R_pid.reset();
				Dir_pid.reset();
				continue;
			}
			tick = System::Time();
			////////////////////PID///////////////////////
			if (tick % 100 == 0) {
				encoder1.Update();
				L_count = encoder1.GetCount();
				encoder2.Update();
				R_count = encoder2.GetCount();
				/////////////Left motor///////////////////
				L_speed = L_pid.output(L_target_count, L_count);
				if (L_speed < 0) {
					if (L_motor.IsClockwise())
						L_motor.SetClockwise(false);
					L_speed = -L_speed;
				} else if (!L_motor.IsClockwise())
					L_motor.SetClockwise(true);
				if (L_speed > 1000)
					L_speed = 1000;
				L_motor.SetPower(L_speed);
				////////////Right motor///////////////////
				R_speed = R_pid.output(R_target_count, -R_count);
				if (R_speed < 0) {
					if (!R_motor.IsClockwise())
						R_motor.SetClockwise(true);
					R_speed = -R_speed;
				} else if (R_motor.IsClockwise())
					R_motor.SetClockwise(false);
				if (R_speed > 1000)
					R_speed = 1000;
				R_motor.SetPower(R_speed);
				////////////////////Debug///////////////////
				char data[20] = { };
				sprintf(data, "E:%d,%d\n", L_count, R_count);
				bt.SendStr(data); /**/
				sprintf(data, "T:%d,%d\n", L_target_count, R_target_count);
				bt.SendStr(data); /**/
				if (!sent) {
					sprintf(data, "S:%d\n", state);
					bt.SendStr(data);
					sent = true;
				}
//				if (!pos_sent) {
//					sprintf(data, "P:%d,%d\n", last_beacon.center.first,
//							last_beacon.center.second);
//					bt.SendStr(data);
//					pos_sent = true;
//				}
				////////////////////////////////////////////
			}
			///////////////////decision//////////////////////
			if (tick % 25 == 0) {
				const Byte* buf = cam.LockBuffer();
//				lcd.SetRegion(Lcd::Rect(0, 0, width, height));
//				lcd.FillBits(0, 0xFFFF, buf, width * height);
				////////////init value///////////////////////
				Beacon beacons[max_beacon];
				uint8_t beacon_count = 0;
				target = NULL;
				///////////////process image/////////////////
				process(buf, beacons, beacon_count);
				///////////////decision making///////////////
//				if (target == NULL && beacon_count) {//have possible beacon but not met requirement
//					/////record the beacon object with the highest density////
//					center_record[frame_count] = Beacon(beacons[0]);
//					for (int i = 1; i < beacon_count; i++) {
//						if (beacons[i].density
//								> center_record[frame_count].density)
//							center_record[frame_count] = Beacon(beacons[i]);
//					}
//					frame_count++;
//					/////if 5 consecutive frames have possible beacon but not sure///////
//					if (frame_count == 5) {
//						int temp = 0;
//						for (int i = 0; i < frame_count - 1; i++) {
//							if (check_near(center_record[i],
//									center_record[i + 1]))
//								temp++;
//						}
//						if (check_near(center_record[frame_count - 1],
//								center_record[0]))
//							temp++;
//						if (temp >= 3) {
//							target = center_record + (frame_count - 1);
//							frame_count = 0;
//						}
//					}
//					if (frame_count == 10) {
//						frame_count = 0;
//					}
//				}
				if (target != NULL) {		//target find
					frame_count = 0;
					pos_sent = false;
					if (rotate == performing)
						rotate = no;
					if (target->area > near_area && rotate != prepare)
						rotate = prepare;
					int diff = Dir_pid.output(200, target->center.first);
					if (diff > 0) {
						if (state != 0) {
							state = 0;
							sent = false;
						}
						R_target_count = forward_speed + diff;
						L_target_count = forward_speed;
					} else {
						if (state != 1) {
							state = 1;
							sent = false;
						}
						R_target_count = forward_speed;
						L_target_count = forward_speed + abs(diff);
					}
					last_beacon.area = target->area;
					last_beacon.center = target->center;
					if (!seen)
						seen = true;
					if (start)
						start = 0;
				} else if (rotate == performing) {
				} else if (seen) { //target not find but have seen target before
					if (start == 0)
						start = System::Time();
					else if (tick - start > 75) {
						if (rotate == prepare) {		//went over the target
							rotate = performing;
							if (last_beacon.center.first < 170) {
								R_target_count = rotate_speed;
								L_target_count = 100;
								if (state != 2) {
									state = 2;
									sent = false;
								}
							} else {
								R_target_count = 100;
								L_target_count = rotate_speed;
								if (state != 3) {
									state = 3;
									sent = false;
								}
							}
						}
						seen = false;
						start = 0;
						if (state != 4 && state != 3 && state !=2) {
							state = 4;
							sent = false;
						}
					}
				} else { //target not find and have not seen target before
					L_target_count = finding_speed;
					R_target_count = finding_speed;
					if (state != 5) {
						state = 5;
						sent = false;
					}
				}
				cam.UnlockBuffer();
//				if (beacon_count) {
//					lcd.SetRegion(Lcd::Rect(target->left_x, 0, 1, height));
//					lcd.FillColor(lcd.kGreen);
//					lcd.SetRegion(Lcd::Rect(target->right_x, 0, 1, height));
//					lcd.FillColor(lcd.kGreen);
//					lcd.SetRegion(Lcd::Rect(0, target->upper_y, width, 1));
//					lcd.FillColor(lcd.kGreen);
//					lcd.SetRegion(Lcd::Rect(0, target->lower_y, width, 1));
//					lcd.FillColor(lcd.kGreen);
//					lcd.SetRegion(
//							Lcd::Rect(target->center.first,
//									target->center.second, 1, 1));
//					lcd.FillColor(lcd.kBlue);
//				}
//				end = System::Time();
//				char data[10] = { };
//				sprintf(data, "%d", end - start);
//				lcd.SetRegion(Lcd::Rect(0, 0, 80, 15));
//				writer.WriteBuffer(data, 10);
//				lcd.SetRegion(Lcd::Rect(0, 105, 80, 15));
//				sprintf(data, "%d", target_count);
//				writer.WriteBuffer(data, 10);
			}
		}
	}
	return 0;
}
