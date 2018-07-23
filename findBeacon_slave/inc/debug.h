/*
 *
 *  Created on: Jun 16, 2018
 *      Author: Sheldon
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_
#include "var.h"
#include "image_processing.h"

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

void display(Beacon temp, uint16_t color) {
	lcd->SetRegion(
			Lcd::Rect(temp.left_x - offset, temp.upper_y,
					temp.right_x - temp.left_x, temp.lower_y - temp.upper_y));
	lcd->FillColor(color);
}

inline void display_num(PkgType t) {
	char out[20] = { };
	Beacon *ptr = NULL;
	switch (t) {
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

void print_line(std::list<point>::iterator begin,
		std::list<point>::iterator end) {
	for (; begin != end; begin++) {
		lcd->SetRegion(Lcd::Rect(begin->x, begin->y, 1, 1));
		lcd->FillColor(Lcd::kBlue);
	}
}

inline void show_avoid_region() {

	//#scan region

	int len = avoid_region_up.right_x - avoid_region_up.left_x;
//	lcd->SetRegion(
//			Lcd::Rect(avoid_region_up.left_x, avoid_region_up.lower_y, len, 1));
//	lcd->FillColor(Lcd::kRed);
//	len = avoid_region_up.lower_y - avoid_region_up.upper_y;
//	lcd->SetRegion(Lcd::Rect(avoid_region_up.right_x - offset, 0, 1, len));
//	lcd->FillColor(Lcd::kRed);
//	lcd->SetRegion(Lcd::Rect(avoid_region_up.left_x - offset, 0, 1, len));
//	lcd->FillColor(Lcd::kRed);
//
//	len = avoid_region_left.lower_y - avoid_region_left.upper_y;
//	lcd->SetRegion(
//			Lcd::Rect(avoid_region_left.right_x - offset,
//					avoid_region_left.upper_y, 1, len));
//	lcd->FillColor(Lcd::kRed);
//	len = avoid_region_right.lower_y - avoid_region_right.upper_y;
//	lcd->SetRegion(
//			Lcd::Rect(avoid_region_right.left_x - offset,
//					avoid_region_right.upper_y, 1, len));
//	lcd->FillColor(Lcd::kRed);
//	len = avoid_region_left.right_x - avoid_region_left.left_x;
//	lcd->SetRegion(
//			Lcd::Rect(avoid_region_left.left_x, avoid_region_left.upper_y, len,
//					1));
//	lcd->FillColor(Lcd::kRed);
//	len = avoid_region_right.right_x - avoid_region_right.left_x;
//	lcd->SetRegion(
//			Lcd::Rect(avoid_region_right.left_x, avoid_region_right.upper_y,
//					len, 1));
//	lcd->FillColor(Lcd::kRed);
//
//	//#car head
	len = no_scan.right_x - no_scan.left_x;
	lcd->SetRegion(Lcd::Rect(no_scan.left_x - offset, no_scan.upper_y, len, 1));
	lcd->FillColor(Lcd::kRed);
	len = no_scan.lower_y - no_scan.upper_y;
	lcd->SetRegion(Lcd::Rect(no_scan.left_x - offset, no_scan.upper_y, 1, len));
	lcd->FillColor(Lcd::kRed);
	lcd->SetRegion(
			Lcd::Rect(no_scan.right_x - offset, no_scan.upper_y, 1, len));
	lcd->FillColor(Lcd::kRed);

}

inline void display_state(working_mode m)
{
	switch (m)
	{
		case image:
		display_greyscale_image();
//		lcd->SetRegion(Lcd::Rect(0,30,160,90));
//		lcd->FillColor(Lcd::kWhite);
		show_avoid_region();
		if (irState == checked && System::Time() - find_time > 100)
			display(*ir_target, Lcd::kRed);
		if (o_target != NULL)
			display(*o_target, Lcd::kBlue);
		break;
		case word:
		if (irState == checked && System::Time() - find_time > 100)
		display_num(PkgType::irTarget);
		else
		{
			lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
//			lcd->ClearRegion();
			writer->WriteString("no");
		}
		if (o_target != NULL)
		display_num(PkgType::oTarget);
		else
		{
			lcd->SetRegion(Lcd::Rect(0, 45, 160, 15));
//			lcd->ClearRegion();
			writer->WriteString("no");
		}

		break;
	}
}

#endif /* INC_DEBUG_H_ */
