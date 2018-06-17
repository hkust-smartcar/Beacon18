/*
 * debug.h
 *
 *  Created on: Jun 16, 2018
 *      Author: Sheldon
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

void send_image(bool comfirm) {
	int size = cam->GetH() * cam->GetW();
	buf = cam->LockBuffer();
	bt->SendBuffer(buf, size / 3);
	while (!comfirm)
		;
	comfirm = false;
	bt->SendStrLiteral("\n");
	bt->SendBuffer(buf + size / 3, size / 3);
	while (!comfirm)
		;
	comfirm = false;
	bt->SendStrLiteral("\n");
	bt->SendBuffer(buf + size / 3 * 2, size / 3);
	while (!comfirm)
		;
	comfirm = false;
	bt->SendStrLiteral("\n");
	cam->UnlockBuffer();
}

inline void display_time(int start) {
	char data[20] = { };
	sprintf(data, "%d", System::Time() - start);
	lcd->SetRegion(Lcd::Rect(0, 100, 160, 15));
	writer->WriteBuffer(data, 20);
}

inline void show_avoid_region() {
	lcd->SetRegion(Lcd::Rect(80, 0, 1, cam->GetH()));
	lcd->FillColor(Lcd::kRed);
	lcd->SetRegion(Lcd::Rect(130, 0, 1, cam->GetH()));
	lcd->FillColor(Lcd::kRed);
	lcd->SetRegion(Lcd::Rect(0, 20, 160, 1));
	lcd->FillColor(Lcd::kRed);
}

inline void display_greyscale_image() {
	for (uint i = 0; i < height; i++) {
		lcd->SetRegion(Lcd::Rect(0, i, 160, 1));
		lcd->FillGrayscalePixel(buf + width * i, 160);
	}
}

void display(Beacon temp, uint16_t color) {
	lcd->SetRegion(
			Lcd::Rect(temp.left_x, temp.upper_y, temp.right_x - temp.left_x,
					temp.lower_y - temp.upper_y));
	lcd->FillColor(color);
}

inline void display_num(PkgType t) {
	char out[20] = { };
	Beacon* ptr = NULL;
	switch (t) {
	case sameTarget:
		lcd->SetRegion(Lcd::Rect(0, 60, 160, 15));
		writer->WriteString("sameTarget");
		lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
		ptr = ir_target;
		break;
	case irTarget:
		ptr = ir_target;
		lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
		break;
	case oTarget:
		ptr = o_target;
		lcd->SetRegion(Lcd::Rect(0, 45, 160, 15));
		break;
	}
	sprintf(out, "%d , %d", ptr->center.first, ptr->center.second);
	writer->WriteBuffer(out, 20);
}

#endif /* INC_DEBUG_H_ */
