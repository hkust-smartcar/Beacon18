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
//#include "libsc/joystick.h"
#include "libsc/battery_meter.h"
//#include "libbase/k60/pit.h"
#include "libbase/misc_utils_c.h"
#include "image_processing.h"
#include "var.h"
#include "config.h"

namespace libbase {
namespace k60 {

Mcg::Config Mcg::GetMcgConfig() {
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}
} // namespace k60
} // namespace libbase

using libsc::System;
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;

void reset_recrod() {
	if (ir_target != NULL) {
		if (ir_record != NULL)
			delete ir_record;
		ir_record = new Beacon(*ir_target);
		delete ir_target;
		ir_target = NULL;
	}

	if (o_target != NULL) {
		if (o_record != NULL)
			delete o_record;
		o_record = new Beacon(*o_target);
		delete o_target;
		o_target = NULL;
	}
}

enum PkgType {
	irTarget = 0, oTarget = 1, sameTarget = 2
};

struct BitConsts {
	 uint8_t kSTART = 0xF0;
	 uint8_t kEND = 0xFF;
};
void send_coord(uint8_t type) {
	BitConsts a;
	uint8_t buffer[5];
	buffer[0] = a.kSTART;
	Beacon* ptr = NULL;
	switch (type) {
	case PkgType::irTarget:
		buffer[1] = PkgType::irTarget;
		ptr = ir_target;
		break;
	case PkgType::oTarget:
		buffer[1] = PkgType::oTarget;
		ptr = o_target;
		break;
	case PkgType::sameTarget:
		buffer[1] = PkgType::sameTarget;
		ptr = ir_target;
		break;
	}
	buffer[2] = (uint8_t)ptr->center.first;
	buffer[3] = ptr->center.second;
	buffer[4] = a.kEND;
	comm->SendBuffer(buffer, 5);
}

// void send_image(){
//		if (tick != System::Time() && run) {
//			tick = System::Time();
//			if (tick % 300 == 0) {
//				int size = cam->GetH() * cam->GetW();
//				buf = cam->LockBuffer();
//				bt.SendBuffer(buf, size / 3);
//				while (!comfirm)
//					;
//				comfirm = false;
//				bt.SendStrLiteral("\n");
//				bt.SendBuffer(buf + size / 3, size / 3);
//				while (!comfirm)
//					;
//				comfirm = false;
//				bt.SendStrLiteral("\n");
//				bt.SendBuffer(buf + size / 3 * 2, size / 3);
//				while (!comfirm)
//					;
//				comfirm = false;
//				bt.SendStrLiteral("\n");
//				cam->UnlockBuffer();
//			}
//				char data[20] ={};
//				sprintf(data,"%d",System::Time() - start);
//				lcd->SetRegion(Lcd::Rect(0,0,160,15));
//				writer.WriteBuffer(data,20);
//		}
//	}
// }

void display_time(uint16_t start) {
	char data[20] = { };
	sprintf(data, "%d", System::Time() - start);
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
	writer->WriteBuffer(data, 20);
}

int main() {
	System::Init();

	Led Led0(init_led(0));
	led0 = &Led0;
	Led Led1(init_led(1));
	led1 = &Led1;
	St7735r lcd_(init_lcd());
	lcd = &lcd_;
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd->Clear(Lcd::kWhite);
	LcdTypewriter writer_(init_writer());
	writer = &writer_;
	k60::MT9V034 cam_(init_cam());
	cam = &cam_;
	width = cam->GetW();
	height = cam->GetH();
	bool run = false;
	bool comfirm = false;
	JyMcuBt106 bt_(init_bt(run, comfirm));
	bt = &bt_;
	JyMcuBt106 comm_(init_comm(run));
	comm = &comm_;
	int start = 0;
	uint32_t tick = System::Time();
	cam->Start();
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
	writer->WriteString("ir target");
	lcd->SetRegion(Lcd::Rect(0, 30, 160, 15));
	writer->WriteString("o target");
	while (1) {
		if (tick != System::Time()) {
			tick = System::Time();
			if (tick % 25 == 0) {
//				start = tick;
				buf = cam->LockBuffer();
//			for (uint i = 0; i < height; i++) {
//				lcd->SetRegion(Lcd::Rect(0, i, 160, 1));
//				lcd->FillGrayscalePixel(buf + cam->GetW() * i, 160);
//			}
				process();
				char out[20] = { };
				Beacon* ptr = NULL;
				if (same) {
					send_coord(PkgType::sameTarget);
					lcd->SetRegion(Lcd::Rect(0, 60, 160, 15));
					writer->WriteString("sameTarget");
					ptr = ir_target;
					lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
					sprintf(out, "%d , %d", ptr->center.first,
							ptr->center.second);
					writer->WriteBuffer(out, 20);
				}
//					display(*ir_target, Lcd::kGreen);
				else {
					if (ir_target != NULL) {
//					display(*ir_target, Lcd::kRed);
						send_coord(PkgType::irTarget);
						ptr = ir_target;
						lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
						sprintf(out, "%d , %d", ptr->center.first,
								ptr->center.second);
						writer->WriteBuffer(out, 20);
					}
					if (o_target != NULL) {
//					display(*o_target, Lcd::kBlue);
						send_coord(PkgType::oTarget);
						ptr = o_target;
						lcd->SetRegion(Lcd::Rect(0, 45, 160, 15));
						sprintf(out, "%d , %d", ptr->center.first,
								ptr->center.second);
						writer->WriteBuffer(out, 20);
					}
				}
				if (o_target == NULL && ir_target == NULL) {
					lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
					writer->WriteString("No");
					lcd->SetRegion(Lcd::Rect(0, 45, 160, 15));
										writer->WriteString("No");
				}
				reset_recrod();
				cam->UnlockBuffer();
//				display_time(start);
			}
		}
	}
	return 0;
}
