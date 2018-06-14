/*
 * config.h
 *
 *  Created on: Jun 11, 2018
 *      Author: Sheldon
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
#include "var.h"
#include <list>

Led::Config init_led(uint16_t id) {
	Led::Config led_config;
	led_config.is_active_low = true;
	led_config.id = id;
	return led_config;
}

St7735r::Config init_lcd() {
	St7735r::Config lcd_config;
	lcd_config.orientation = 1;
	lcd_config.fps = 60;
	return lcd_config;
}

LcdTypewriter::Config init_writer() {
	LcdTypewriter::Config writer_config;
	writer_config.lcd = lcd;
	return writer_config;
}

Ov7725::Config init_cam() {
	Ov7725::Config cam_config;
	cam_config.id = 0;
	cam_config.w = width;
	cam_config.h = height;
	cam_config.contrast = contrast;
	cam_config.brightness = brightness;
	cam_config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	return cam_config;
}

JyMcuBt106::Config init_bt(bool& run) {
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 1;
	config.rx_isr = [&run](const Byte *data, const size_t size) {
		if (data[0] == 's')
		{
			run = true;
			led0->SetEnable(1);
			comm->SendStrLiteral("s");
		}
		if (data[0] == 'S')
		{
			run = false;
			led0->SetEnable(0);
			L_motor->SetPower(0);
			R_motor->SetPower(0);
			comm->SendStrLiteral("s");
		}
		return true;
	};
	return config;
}

enum PkgType {
	irTarget = 0, oTarget = 1, sameTarget = 2
};
struct BitConsts {
	uint8_t kSTART = 0xF0;
	uint8_t kEND = 0xFF;
};
std::list<uint8_t> buffer;

inline void BuildBufferPackage() {
	auto it = buffer.begin();
	uint8_t type = *(it++);
	uint8_t x = *(it++);
	uint8_t y = *(it++);
	BeaconPackage* ptr = NULL;
	switch (type) {
	case PkgType::sameTarget:
		ir_target2.same = true;
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
JyMcuBt106::Config init_comm() {
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k4800;
	config.id = 2;
	config.rx_isr =
			[](const Byte *data, const size_t size) {
			BitConsts a;
				if(data[0] == a.kSTART)
				buffer.clear();
				else if(data[0] == a.kEND)
				BuildBufferPackage();
				else
				buffer.push_back(data[0]);
				return true;
			};
	return config;
}

AlternateMotor::Config init_motor(int id) {
	AlternateMotor::Config motor_config;
	motor_config.id = id;
	return motor_config;
}

DirEncoder::Config init_encoder(int id) {
	DirEncoder::Config encoder_config;
	encoder_config.id = id;
	return encoder_config;
}
#endif /* INC_CONFIG_H_ */
