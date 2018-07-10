/*
 * debug.h
 *
 *  Created on: Jun 16, 2018
 *      Author: Sheldon
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_
#include "var.h"

void send_image(bool comfirm)
{
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

inline void display_time(int start)
{
	char data[20] = {};
	sprintf(data, "%d", System::Time() - start);
	lcd->SetRegion(Lcd::Rect(0, 100, 160, 15));
	writer->WriteBuffer(data, 20);
}

void display(Beacon temp, uint16_t color)
{
	lcd->SetRegion(
		Lcd::Rect(temp.left_x - offset, temp.upper_y, temp.right_x - temp.left_x,
				  temp.lower_y - temp.upper_y));
	lcd->FillColor(color);
}

inline void display_num(PkgType t)
{
	char out[20] = {};
	Beacon *ptr = NULL;
	switch (t)
	{
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

inline void display_state(working_mode m)
{
	switch (m)
	{
	case image:
		display_greyscale_image();
//		lcd->SetRegion(Lcd::Rect(0,0,160,120));
//		lcd->FillColor(Lcd::kWhite);
		show_avoid_region();
//		if (irState == checked && System::Time() - find_time > 100)
//			display(*ir_target, Lcd::kRed);
//		if (o_target != NULL)
//			display(*o_target, Lcd::kBlue);
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
