/*
 * config.h
 *
 *  Created on: Jun 11, 2018
 *      Author: Sheldon
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
#include "var.h"
#include "function.h"
#include <list>
#include "libbase/k60/vectors.h"

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

BatteryMeter::Config init_bMeter(){
	BatteryMeter::Config bConfig;
	bConfig.voltage_ratio = 0.4;
	return bConfig;
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

JyMcuBt106::Config init_bt() {
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 0;
	config.rx_isr = &bt_listener;
	return config;
}

JyMcuBt106::Config init_comm() {
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k9600;
	config.id = 2;
	config.rx_isr = &comm_listener;
	// NVIC_SetPriority(UART5_RX_TX_IRQn, __BASE_IRQ_PRIORITY - 3);
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
