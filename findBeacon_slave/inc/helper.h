#ifndef INC_HELPER_H_
#define INC_HELPER_H_

#include "var.h"

void insert(Beacon t, ptr_mode m) {
	Beacon** ptr = NULL;
	switch (m) {
	case o:
		ptr = &o_target;
		break;
	case oRecord:
		ptr = &o_record;
		break;
	case irRecord:
		ptr = &ir_record;
		break;
	case ir_Target:
		ptr = &ir_target;
		break;
	case o_Target:
		ptr = &o_target;
		break;
	}
	if (*ptr != NULL) {
		delete *ptr;
		*ptr = NULL;
	}
	*ptr = new Beacon(t);
}
bool comm_listener(const Byte *data, const size_t size) {
	if (data[0] == 's') {
		run = true;
		lcd->Clear(Lcd::kBlack);
	}
	if (data[0] == 'S')
		run = false;
	return true;
}

void send_coord(uint8_t type) {
	BitConsts a;
	uint8_t buffer[7];
	buffer[0] = a.kSTART;
	Beacon *ptr = NULL;
	bool send_line = false;
	switch (type) {
	case PkgType::irTarget:
		buffer[1] = PkgType::irTarget;
		ptr = ir_target;
		break;
	case PkgType::oTarget:
		buffer[1] = PkgType::oTarget;
		ptr = o_target;
		break;
	case PkgType::hLine:
		buffer[1] = PkgType::hLine;
		send_line = true;
		break;
	case PkgType::vLine:
		buffer[1] = PkgType::vLine;
		send_line = true;
		break;
	case PkgType::corner:
		buffer[1] = PkgType::corner;
		send_line = true;
		break;
	}
	if (send_line) {
//		buffer[2] = begin.x;
//		buffer[3] = begin.y;
//		buffer[4] = end.x;
//		buffer[5] = end.y;
		buffer[2] = a.kEND;
		comm->SendBuffer(buffer, 3);
	} else {
		buffer[2] = (uint8_t) ptr->center.first;
		buffer[3] = ptr->center.second;
		buffer[4] = a.kEND;
		comm->SendBuffer(buffer, 5);
	}
}

inline void reset_recrod() {
	if (ir_target != NULL) {
		insert(*ir_target, ptr_mode::irRecord);
		delete ir_target;
		ir_target = NULL;
	}

	if (o_target != NULL) {
		insert(*o_target, ptr_mode::oRecord);
		delete o_target;
		o_target = NULL;
	}
}

inline void display_greyscale_image() {
	for (uint i = 0; i < height; i++) {
		lcd->SetRegion(Lcd::Rect(0, i, 160, 1));
		lcd->FillGrayscalePixel(buf + offset + width * i, 160);
	}
}

inline void check_cam() {
	bool down_timer = false;
	uint16_t down_time = 0;
	while (!cam->IsAvailable())
		;
	buf = cam->LockBuffer();
	display_greyscale_image();
	while (true) {
		if (joystick->GetState() == Joystick::State::kIdle && !down_timer)
			continue;
		if (joystick->GetState() == Joystick::State::kSelect) {
			if (!down_timer) {
				down_timer = true;
				down_time = System::Time();
			}
		} else if (down_timer && System::Time() - down_time > 500) {
			break;
		} else {
			cam->RegSet(0x0C, 0x03);
			cam->UnlockBuffer();
			System::DelayMs(300);
			buf = cam->LockBuffer();
			display_greyscale_image();
			down_timer = false;
		}
	}
	cam->UnlockBuffer();
	lcd->SetRegion(Lcd::Rect(0, 0, 160, 128));
	lcd->Clear(Lcd::kWhite);
}

#endif /* INC_HELPER_H_ */
