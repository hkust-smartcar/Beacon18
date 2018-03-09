/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <string>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/dir_encoder.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
//#include "libsc/joystick.h"
#include "libsc/st7735r.h"
//#include "libsc/battery_meter.h"
#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include <libsc/k60/ov7725.h>
#include "libsc/alternate_motor.h"

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

float kp = 0.1;
float kd = 0.9;
float ki = 0.005;
const uint16_t width = 240;
const uint16_t height = 180;

//uint8_t process(const Byte* buf, uint16_t temp[]) {
//	uint16_t pos = 0;
//	int bit_pos = 8;
//	uint8_t position = 0;
//	for (uint16_t y = 0; y < height; y++) {
//		for (uint16_t x = 0; x < width; ++x) {
//			if (--bit_pos < 0) {
//				bit_pos = 7;
//				++pos;
//			}
//			if (!GET_BIT(buf[pos], bit_pos)){
//				temp[position++] = x;
//				temp[position++] = y;
//			}
//		}
//	}
//	return position;
//}

int main() {

	System::Init();
	//led init
	Led::Config led_config;
	led_config.is_active_low = true;
	led_config.id = 0;
	Led led0(led_config);
	led_config.id = 1;
	Led led1(led_config);
	led_config.id = 2;
	Led led2(led_config);
	led_config.id = 3;
	Led led3(led_config);

	St7735r::Config lcd_config;
	lcd_config.orientation = 0;
	lcd_config.fps = 20;
	St7735r lcd(lcd_config);

	LcdTypewriter::Config writer_config;
	writer_config.lcd = &lcd;
	LcdTypewriter writer(writer_config);

	uint32_t tick = System::Time();
	bool c_start = false;
	bool g_start = false;
	bool receiving[] = { false, false, false };
	int8_t factor = 0;
	int contrast = 0x40;
	int brightness = 0x00;
	uint32_t temp_data = 0;

	AlternateMotor::Config motor_config;
	motor_config.id = 0;
	AlternateMotor motor1(motor_config);
	motor_config.id = 1;
	AlternateMotor motor2(motor_config);

	Ov7725::Config cam_config;
	cam_config.id = 0;
	cam_config.w = width;
	cam_config.h = height;
	cam_config.contrast = contrast;
	cam_config.brightness = brightness;
	cam_config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 cam(cam_config);

	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 0;
	config.tx_buf_size = 2; //change this to 1 if working with large image size
	JyMcuBt106 bt(config);
	bt.SetRxIsr(
			[&lcd,&led0,&g_start,&c_start,&receiving,&led1,&brightness,&cam,&bt,&contrast,&motor1,&motor2,&factor,&temp_data,&writer](const Byte *data, const size_t size) {
				if(data[0] =='c') {
					led0.Switch();
					c_start = !c_start;
					if(c_start) {
						cam.Start();
						char temp[20] = {};
						sprintf(temp,"%d\n%d\n",width,width * height /8);
						bt.SendStr(temp);
					}
					else
					cam.Stop();
				}
				if(data[0] =='g') {
					char temp[30] = {};
					led1.Switch();
					g_start = !g_start;
					sprintf(temp,"%f\n%f\n%f\n",kp,ki,kd);
					bt.SendStr(temp);
					motor1.SetPower(200);
					motor2.SetPower(200);
				}
				if(data[0]=='s') {
					c_start = false;
					g_start = false;
					led0.SetEnable(0);
					led1.SetEnable(0);
					motor1.SetPower(0);
					motor2.SetPower(0);
				}
				if(data[0]== 'a') {
					const Byte* buf = cam.LockBuffer();
					bt.SendBuffer(buf, width * height /8);
					cam.UnlockBuffer();
				}
				if(data[0] == 'q') {
					brightness+= 0x01;
					cam.ChangeSecialDigitalEffect(brightness,contrast);
				}
				if(data[0] == 'w') {
					brightness-= 0x01;
					cam.ChangeSecialDigitalEffect(brightness,contrast);
				}
				if(data[0] == 'e') {
					contrast+= 0x01;
					cam.ChangeSecialDigitalEffect(brightness,contrast);
				}
				if(data[0] == 'r') {
					contrast-= 0x01;
					cam.ChangeSecialDigitalEffect(brightness,contrast);
				}
				if(receiving[0] == true) {
					bt.SendStr("\n");
					temp_data |= data[0] << factor;
					factor -= 8;
					if(factor < 0 ) {
						receiving[0] = false;
						memcpy(&kp,&temp_data,sizeof(kp));
						char out[20]= {};
						sprintf(out,"kp: %f",kp);
						lcd.SetRegion(Lcd::Rect(0,0,100,15));
						writer.WriteBuffer(out,10);
					}
				}
				if(receiving[1] == true) {
					bt.SendStr("\n");
					temp_data |= data[0] << factor;
					factor -= 8;
					if(factor < 0 ) {
						receiving[1] = false;
						memcpy(&ki,&temp_data,sizeof(ki));
						char out[20]= {};
						sprintf(out,"ki: %f",ki);
						lcd.SetRegion(Lcd::Rect(0,16,100,15));
						writer.WriteBuffer(out,10);
					}
				}
				if(receiving[2] == true) {
					bt.SendStr("\n");
					temp_data |= data[0] << factor;
					factor -= 8;
					if(factor < 0 ) {
						receiving[2] = false;
						memcpy(&kd,&temp_data,sizeof(kd));
						char out[20]= {};
						sprintf(out,"kd: %f",kd);
						lcd.SetRegion(Lcd::Rect(0,32,100,15));
						writer.WriteBuffer(out,10);
					}
				}

				if(data[0] == 'p') {
					temp_data = 0;
					receiving[0] = true;
					factor = 24;
					bt.SendStr("\n");
				}
				if(data[0] == 'i') {
					temp_data = 0;
					receiving[1] = true;
					factor = 24;
					bt.SendStr("\n");
				}
				if(data[0] == 'd') {
					temp_data = 0;
					receiving[2] = true;
					factor = 24;
					bt.SendStr("\n");
				}
				return true;
			});

	DirEncoder::Config encoder_config;
	encoder_config.id = 1;
	DirEncoder encoder1(encoder_config);
//	ab_config.id = 1;
//	AbEncoder encoder2(ab_config);

	lcd.SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd.Clear(Lcd::kWhite);
	int count1 = 0;
	int count2 = 0;

//	int start_time = 0;
//	int end = 0;
//	bool img[height][width];
//	uint16_t temp[100];

	while (1) {
//		char data[5] = { };
//		start_time = System::Time();
//		end = System::Time();
//		sprintf(data, "%d\r\n", end - start_time);
//		bt.SendStr(data);
		if (tick != System::Time()) {
			tick = System::Time();
			if (c_start && tick % 50 == 0) {
				const Byte* buf = cam.LockBuffer();
				bt.SendBuffer(buf, width * height / 8);
				cam.UnlockBuffer();
			}
			if (tick % 100 == 0 && g_start) {
				char data[20] = { };
				encoder1.Update();
//				encoder2.Update();
				count1 = encoder1.GetCount();
//				count2 = encoder2.GetCount();

				sprintf(data, "%d\n", count1);
//				lcd.SetRegion(Lcd::Rect(0, 0, 128, 15));
//				writer.WriteBuffer(data, 20);
				std::string out = data;
				bt.SendStr(data);
			}
		}
	}
	return 0;
}
