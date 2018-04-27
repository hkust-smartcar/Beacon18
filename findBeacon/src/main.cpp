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

const uint16_t width = 320;
const uint16_t height = 240;
const uint16_t numOfPixel = width * height / 8;

float L_kp = 0.04;
float L_ki = 0.012;
float L_kd = 0.0085;

float R_kp = 0.0359;
float R_ki = 0.012;
float R_kd = 0.0099;

uint8_t contrast = 0x40;
uint8_t brightness = 0x00;

const uint8_t x_range = 5;  //160 = 5
const uint8_t y_range = 55; //160 = 10
const uint16_t min_size = 70; //160 = 40
const uint8_t error = 10;
const uint8_t max_beacon = 10;
const uint16_t critical_density = 65;
Beacon* target = NULL;

bool seen = false;
bool not_find = false;

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
	if (check_target(beacons, beacon_count)) {
		beacon_count++;
		return true;
	}
	return false;
}

bool process(const Byte* buf, Beacon beacons[max_beacon], uint8_t &beacon_count,
		std::pair<uint16_t, uint16_t>* last_center) {

	uint16_t pos = 0;
	int8_t bit_pos = 8;

	if (seen) {
		uint16_t temp_pos = (width * last_center->second) / 8
				+ last_center->first / 8;
		uint16_t temp_bit_pos = 8 - (last_center->first % 8 + 1);
		scan(buf, temp_pos, temp_bit_pos, beacons, beacon_count, 1);
		if (beacons[0].count != 0) {
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

void move(AlternateMotor &motor, int &speed) {

	if (speed < 0) {
		speed = 0;
	}
	if (speed > 500)
		speed = 500;
	motor.SetPower(speed);
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

	St7735r::Config lcd_config;
	lcd_config.orientation = 1;
	lcd_config.fps = 60;
	St7735r lcd(lcd_config);

	LcdTypewriter::Config writer_config;
	writer_config.lcd = &lcd;
	LcdTypewriter writer(writer_config);

	AlternateMotor::Config motor_config;
	motor_config.id = 1;
	AlternateMotor L_motor(motor_config);
	motor_config.id = 0;
	AlternateMotor R_motor(motor_config);
	L_motor.SetClockwise(false);

	DirEncoder::Config encoder_config;
	encoder_config.id = 0;		//right
	DirEncoder encoder1(encoder_config);
	encoder_config.id = 1;		//left
	DirEncoder encoder2(encoder_config);

	Ov7725::Config cam_config;
	cam_config.id = 0;
	cam_config.w = width;
	cam_config.h = height;
	cam_config.contrast = contrast;
	cam_config.brightness = brightness;
	cam_config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 cam(cam_config);
	cam.Start();

	PID L_pid(L_kp, L_ki, L_kd);
	L_pid.errorSumBound = 300;
	PID R_pid(R_kp, R_ki, R_kd);
	R_pid.errorSumBound = 300;
	lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd.Clear(Lcd::kWhite);

	uint32_t tick = System::Time();
	std::pair<uint16_t, uint16_t>* last_center = new std::pair<uint16_t,
			uint16_t>(0, 0);
	uint32_t start;
	uint32_t end;
	int L_count = 0;
	int R_count = 0;
	int L_speed = 0;
	int R_speed = 0;
	int L_target_count = 0;
	int R_target_count = 0;
	bool center = false;
	int frame_count = 0;
	Beacon center_record[10];

	while (1) {
		if (tick != System::Time()) {
			tick = System::Time();
			//PID
			if (tick % 100 == 0) {
				encoder1.Update();
				L_count = encoder1.GetCount();
				encoder2.Update();
				R_count = encoder2.GetCount();
				L_speed += L_pid.output(L_target_count, abs(L_count));
				R_speed += L_pid.output(R_target_count, abs(R_count));
				move(L_motor, L_speed);
				move(R_motor, R_speed);
//				char data[20] = { };
//				sprintf(data, "L_encoder: %d", L_count);
//				lcd.SetRegion(Lcd::Rect(0, 45, 128, 15));
//				writer.WriteBuffer(data, 20);
//				sprintf(data, "R_encoder: %d", R_count);
//				lcd.SetRegion(Lcd::Rect(0, 60, 128, 15));
//				writer.WriteBuffer(data, 20);
//				sprintf(data, "L speed: %d", L_speed);
//				lcd.SetRegion(Lcd::Rect(0, 75, 128, 15));
//				writer.WriteBuffer(data, 20);
//				sprintf(data, "R speed: %d", R_speed);
//				lcd.SetRegion(Lcd::Rect(0, 90, 128, 15));
//				writer.WriteBuffer(data, 20);
			}
			if (tick % 25 == 0) {
//				start = System::Time();
				const Byte* buf = cam.LockBuffer();
//				lcd.SetRegion(Lcd::Rect(0, 0, width, height));
//				lcd.FillBits(0, 0xFFFF, buf, width * height);
				Beacon beacons[max_beacon];
				uint8_t beacon_count = 0;
				target = NULL;
				process(buf, beacons, beacon_count, last_center);

				if (target == NULL && beacon_count) {
					center_record[frame_count] = Beacon(beacons[0]);
					for (int i = 1; i < beacon_count; i++) {
						if (beacons[i].density
								> center_record[frame_count].density)
							center_record[frame_count] = Beacon(beacons[i]);
					}
					frame_count++;

					if (frame_count == 5) {
						int temp = 0;
						for (int i = 0; i < frame_count - 1; i++) {
							if (check_near(center_record[i],
									center_record[i + 1]))
								temp++;
						}
						if (check_near(center_record[frame_count - 1],
								center_record[0]))
							temp++;
						if (temp >= 3) {
							target = center_record + (frame_count - 1);
							frame_count = 0;
						}
					}
					if (frame_count == 10) {
						frame_count = 0;
					}
				}
				if (target != NULL) {
					frame_count = 0;
					if (not_find)		//target find
						not_find = false;
					if (target->center.first > 210) {	//160 = 85
						if (center) {
							L_target_count = 250;
							R_target_count = 600;
						} else if (target->center.first > 280) {
							L_motor.SetClockwise(true);
							R_motor.SetClockwise(true);
							L_target_count = 400;
							R_target_count = 400;
						} else {
							L_motor.SetClockwise(true);
							R_motor.SetClockwise(true);
							L_target_count = 500;
							R_target_count = 500;
						}
					} else if (target->center.first < 170) { 	//160 = 70
						if (center) {
							L_target_count = 600;
							R_target_count = 250;
						} else if (target->center.first < 100) {
							L_motor.SetClockwise(false);
							R_motor.SetClockwise(false);
							L_target_count = 350;
							R_target_count = 350;
						} else {
							L_motor.SetClockwise(false);
							R_motor.SetClockwise(false);
							L_target_count = 500;
							R_target_count = 500;
						}
					} else {
						L_target_count = 400;
						R_target_count = 400;
						L_motor.SetClockwise(false);
						R_motor.SetClockwise(true);
						center = true;
					}
					last_center->first = target->center.first;
					last_center->second = target->center.second;
					if (!seen)
						seen = true;
				} else if (seen) { //target not find but have seen target before, do nothing
					if (!not_find) {
						start = System::Time();
						not_find = true;
					} else if (tick - start >= 500) {
						seen = false;
						center = false;
					}
//					lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
//					writer.WriteString("Not find");
				} else { //target not find and have not seen target before, keep rotate
					L_motor.SetClockwise(true);
					R_motor.SetClockwise(true);
					L_target_count = 500;
					R_target_count = 500;
				}
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
//					last_center->first = target->center.first;
//					last_center->second = target->center.second;
//					seen = true;
//					if (state) {
//						end = System::Time();
//						char data[10] = { };
//						sprintf(data, "%d", end - start);
//						lcd.SetRegion(Lcd::Rect(0, 0, 80, 15));
//						writer.WriteBuffer(data, 10);
//						state = false;
//					}
//				} else if (!state) {
//					start = System::Time();
//					state = true;
//					lcd.SetRegion(Lcd::Rect(0, 15, 80, 15));
//					writer.WriteString("not find");
//				}
				cam.UnlockBuffer();
//				end = System::Time();
//				char data[10] = { };
//				sprintf(data, "%d", end - start);
//				lcd.SetRegion(Lcd::Rect(0, 0, 80, 15));
//				writer.WriteBuffer(data, 10);
//				lcd.SetRegion(Lcd::Rect(0, 15, 80, 15));
//				if (beacon_count != 0)
//					writer.WriteString("Find");
//				else
//					writer.WriteString("not find");
//				lcd.SetRegion(Lcd::Rect(0, 105, 80, 15));
//				sprintf(data, "%d", target_count);
//				writer.WriteBuffer(data, 10);
			}
		}
	}
	return 0;
}
