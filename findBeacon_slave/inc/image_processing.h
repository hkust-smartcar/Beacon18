/*
 * image_processing.h
 *
 *  Created on: May 3, 2018
 *      Author: Sheldon
 */

#ifndef INC_IMAGE_PROCESSING_H_
#define INC_IMAGE_PROCESSING_H_

#include "var.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;

uint16_t low_time = 0;
bool timer = false;

const uint8_t size = 3;
const uint8_t white = 210;
int8_t y_mask[3][3] = { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
int8_t x_mask[3][3] = { { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };
Beacon avoid_region(80, 130, 80, 120);
uint8_t min_area = 10;
bool same = false;
uint16_t near_dist = 20;

int16_t cal_sobel(uint16_t x, uint16_t y) {
	int16_t x_mean = 0;
	int16_t y_mean = 0;
	y--;
	x--;
	for (uint16_t i = y; i < height && i < y + size; i++) {
		for (uint8_t z = 0; z < 3; z++) {
			if (x + z >= width)
				break;
			else {
				y_mean += buf[i * width + x + z] * y_mask[i - y][z];
				x_mean += buf[i * width + x + z] * x_mask[i - y][z];
			}
		}
	}
	return abs(x_mean) + abs(y_mean);
}

uint8_t cal_mean(uint16_t x, uint16_t y) {
	uint16_t mean = 0;
	for (uint16_t i = y; i < height && i < y + size; i++) {
		for (uint8_t z = 0; z < size; z++) {
			if (x + z >= width)
				break;
			else {
				mean += buf[i * width + x + z];
			}
		}
	}
	return mean / (size * size);
}

// 0 = beacon, 1 = check o_recrod, 2 = avoid
Beacon check_beacon_edge(int mode) {
	Beacon temp(1000, 0, 1000, 0);
	Beacon* ptr = NULL;
	uint8_t error = 10;
	uint8_t error1 = 15;
	switch (mode) {
	case 0:
		ptr = ir_record;
		break;
	case 1:
		ptr = o_record;
		break;
	case 2:
		ptr = &avoid_region;
		error = 0;
		break;
	}
	int16_t x_start = ptr->left_x - error;
	int16_t x_bound = ptr->right_x + error;
	int16_t y_start = ptr->upper_y - error;
	int16_t y_bound = ptr->lower_y + error;
	if (mode < 2) {
		if (x_start < 0)
			x_start = 0;
		if (x_bound > height)
			x_bound = height;
		if (y_start < 0)
			y_start = 0;
		if (y_bound > height)
			y_bound = height;
	}
	for (uint16_t y = y_start; y < y_bound; y += 3) {
		for (uint16_t x = x_start; x < x_bound; x += 3) {
			if (cal_sobel(x, y) > sobel_value) {
				if (x < temp.left_x
						&& ((mode != 2 || x > temp.left_x - error1)
								|| temp.left_x == 1000))
					temp.left_x = x;
				else if (x > temp.right_x
						&& ((mode != 2 || x < temp.right_x + error1)
								|| temp.right_x == 0))
					temp.right_x = x;
				if (y < temp.upper_y
						&& ((mode != 2 || y > temp.upper_y - error1)
								|| temp.upper_y == 1000))
					temp.upper_y = y;
				else if (y > temp.lower_y
						&& ((mode != 2 || y < temp.lower_y + error1)
								|| temp.lower_y == 0))
					temp.lower_y = y;
				temp.count++;
			}
		}
	}
	if (temp.count < 20)
		temp = Beacon(0, 0);
	else
		temp.calc();
	return temp;
}

Beacon check_beacon_ir(uint16_t x, uint16_t y) {
	Beacon temp(x, y);
	const uint8_t error = 5;
	uint16_t* ptr = NULL;
	uint16_t* moving_ptr = NULL;
	int8_t action = 0;
	for (uint8_t mode = 1; mode < 5; mode++) { //1 right,2 left,3 lower, 4 upper
		uint16_t x_ptr = x;
		uint16_t y_ptr = y;
		uint8_t count = 0;
		switch (mode) {
		case 1:
			ptr = &temp.right_x;
			moving_ptr = &x_ptr;
			action = 1;
			break;
		case 2:
			ptr = &temp.left_x;
			moving_ptr = &x_ptr;
			action = -1;
			break;
		case 3:
			ptr = &temp.lower_y;
			moving_ptr = &y_ptr;
			action = 1;
			break;
		case 4:
			ptr = &temp.upper_y;
			moving_ptr = &y_ptr;
			action = -1;
			break;
		}
		while (true) {
			if (buf[y_ptr * width + x_ptr] > white) { //white pixel
				int var = *moving_ptr + action;
				if (var > 0
						&& ((mode > 2 && var < height)
								|| (mode < 3 && var < width))) { //within boarder continue
					*ptr = var;
					*moving_ptr += (mode % 2 == 0 ? -1 : 1);
				} else
					//out boarder, break
					break;
			} else if (++count < error)
				break;

		}
	}
	temp.calc();
	return temp;
}

bool check_skip(uint16_t x, uint16_t y) {
	if (ir_target == NULL)
		return false;
	uint8_t error = 0;
	if (x < ir_target->right_x + error && y < ir_target->lower_y + error
			&& y
					> (ir_target->upper_y - error < 0 ?
							0 : ir_target->upper_y - error)
			&& x
					> (ir_target->left_x - error < 0 ?
							0 : ir_target->left_x - error))
		return true;
	return false;
}

void display(Beacon temp, uint16_t color) {
	lcd->SetRegion(
			Lcd::Rect(temp.left_x, temp.upper_y, temp.right_x - temp.left_x,
					temp.lower_y - temp.upper_y));
	lcd->FillColor(color);
}
//0 upper, 1 lower left, 2 lower right
bool loop(uint8_t mode) {
	Beacon temp;
	uint16_t x;
	uint16_t x_bound;
	uint16_t y;
	uint16_t y_bound;
	switch (mode) {
	case 0:
		x = 0;
		x_bound = width;
		y = 0;
		y_bound = avoid_region.upper_y;
		break;
	case 1:
		x = 0;
		x_bound = avoid_region.left_x;
		y = avoid_region.upper_y;
		y_bound = height;
		break;
	case 2:
		x = avoid_region.left_x;
		x_bound = width;
		y = avoid_region.upper_y;
		y_bound = height;
		break;
	}
	for (; y < y_bound; y += size) {
		for (; x < x_bound; x += size) {
			if (cal_mean(x, y) > white) {
				temp = check_beacon_ir(x, y);
				if (temp.area > min_area) {
					ir_target = new Beacon(temp);
					return true;
				}
			}
		}
	}
	return false;
}

bool check_near(const Beacon b1, const Beacon b2) {
	return abs(b1.center.first - b2.center.first) < near_dist
			&& abs(b1.center.second - b2.center.second) < near_dist;
}

void process() {
	same = false;
	Beacon temp;
	if (timer && System::Time() - low_time > 150) {
		if (ir_record != NULL) {
			delete ir_record;
			ir_record = NULL;
		}
		timer = false;
	}

//	if (ir_record != NULL) {
//		temp = check_beacon_ir(ir_record->center.first,
//				ir_record->center.second);
//		for (int i = 0; i < 2; i++) {
//			if (temp.area > min_area) {
//				if (i == 1) {
//					if (!timer) {
//						low_time = System::Time();
//						timer = true;
//					}
//				} else
//					timer = false;
//				ir_target = new Beacon(temp);
//				if (o_record != NULL) {		//check o_record
//					temp = check_beacon_edge(1);
//					if (temp.area > min_area) {
//						o_target = new Beacon(temp);
//						if (check_near(*o_target, *ir_target))
//							same = true;
//						return;		//ir and o both find -> return
//					}
//				}
//				temp = check_beacon_edge(2);	//check the avoid region
//				if (temp.area > min_area) {
//					o_target = new Beacon(temp);
//					if (check_near(*o_target, *ir_target))
//						same = true;
//					return;			//ir and o both find -> return
//				}
//				break;
//			}
//			temp = check_beacon_edge(0);
//		}
//		timer = false;
//		delete ir_record;
//		ir_record = NULL;
//	}
//	if (o_record != NULL) {		//ir_record must equal to NULL at this point
//		temp = check_beacon_edge(1);
//		if (temp.area > min_area)
//			o_target = new Beacon(temp);	//only o is find -> continue
//		else {
//			delete o_record;
//			o_record = NULL;
//		}
//	}
	temp = check_beacon_edge(2);	//check the avoid region
	if (temp.area > min_area) {
		o_target = new Beacon(temp);
		temp = check_beacon_ir(o_target->center.first, o_target->center.second);//check if that beacon is also a target or not
		if (temp.area > min_area) {
			ir_target = new Beacon(temp);
			same = true;
			return;
		}
	}
	if (loop(0))
		return;
	if (loop(1))
		return;
	if (loop(2))
		return;

//	for (uint16_t y = 0; y < avoid_region.upper_y; y += size) {
//		for (uint16_t x = 0; x < width; x += size) {
////			if (check_skip(x, y)) {
////				x = ir_target->right_x + 1;
////				if (x >= width)
////					break;
////			}
//			if (cal_mean(x, y) > white) {
//				temp = check_beacon_ir(x, y);
//				if (temp.area > min_area)
//					ir_target = new Beacon(temp);
//				return;
//			}
//		}
//	}
}
#endif /* INC_IMAGE_PROCESSING_H_ */
