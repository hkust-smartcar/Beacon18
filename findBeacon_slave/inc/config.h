/*
 * config.h
 *
 *  Created on: Jun 11, 2018
 *      Author: Sheldon
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
#include "var.h"

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

MT9V034::Config init_cam() {
	MT9V034::Config cam_config;
	cam_config.h_binning = cam_config.k4;
	cam_config.w_binning = cam_config.k4;
	return cam_config;
}

JyMcuBt106::Config init_bt(bool& run, bool& comfirm) {
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.id = 0;
	config.tx_buf_size = 1;
	config.rx_isr = [&run,&comfirm](const Byte *data, const size_t size) {
		if(data[0] =='c') {
			led0->Switch();
			run = true;
			char temp[20] = {};
			sprintf(temp,"%d\n%d\n",cam->GetW(),cam->GetH());
			bt->SendStr(temp);
		}
		if(data[0]=='s') {
			run = false;
			cam->Stop();
			led0->SetEnable(0);
		}
		if(data[0] == '\n') {
			comfirm = true;
		}
		return true;
	};
	return config;
}

JyMcuBt106::Config init_comm(bool& run) {
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k9600;
	config.id = 2;
	config.rx_isr = [&run](const Byte *data, const size_t size) {
		if(data[0] == 's') {
			run = !run;
			led0->Switch();
		}
		return true;
	};
	return config;
}
#endif /* INC_CONFIG_H_ */
