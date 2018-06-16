/*
 * image_processing.h
 *
 *  Created on: May 3, 2018
 *      Author: Sheldon
 */

#ifndef INC_IMAGE_PROCESSING_H_
#define INC_IMAGE_PROCESSING_H_

#include "var.h"
#include <list>
#include "math.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;

uint16_t low_time = 0;
bool timer = false;

const uint8_t size = 3;
const uint8_t white = 210;
int8_t y_mask[3][3] = { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
int8_t x_mask[3][3] = { { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };
Beacon avoid_region(80, 130, 20, 100);
uint8_t min_area = 10;
uint16_t near_dist = 10;
uint8_t min_edge = 5;

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

inline bool check_near(const point p1, const point p2) {
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)) < near_dist;
}

bool sort_x(const point& first, const point& second) {
	return first.x < second.x;
}

bool sort_y(const point& first, const point& second) {
	return first.y < second.y;
}

// 0 = beacon, 1 = check o_recrod, 2 = avoid
Beacon check_beacon_edge(int mode) {
	Beacon temp;
	Beacon* ptr = NULL;
	uint8_t error = 10;
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
		if (x_bound > width)
			x_bound = width;
		if (y_start < 0)
			y_start = 0;
		if (y_bound > height)
			y_bound = height;
	}
	std::list<point> edges;
	auto it = edges.begin();
	auto rit = edges.rbegin();
	for (uint16_t y = y_start; y < y_bound; y += 2)
		for (uint16_t x = x_start; x < x_bound; x += 2)
			if (cal_sobel(x, y) > sobel_value) {
				point p(x, y);
//				lcd->SetRegion(Lcd::Rect(x, y, 1, 1));
//				lcd->FillColor(Lcd::kBlue);
				it = edges.begin();
				for (; it != edges.end(); it++) {
					if (check_near(*it, p)) {
						p.count++;
						it->count++;
					}
				}
				edges.push_back(p);
			}
	edges.sort(sort_x);
	for (int a = 0; a < 2; a++) {
		it = edges.begin();
		for (; it != edges.end(); it++)
			if (it->count > min_edge) {
				if (a == 0)
					temp.left_x = it->x;
				else
					temp.upper_y = it->y;
				break;
			}
		rit = edges.rbegin();
		for (; rit != edges.rend(); ++rit)
			if (rit->count > min_edge) {
				if (a == 0)
					temp.right_x = rit->x;
				else
					temp.lower_y = rit->y;
				break;
			}
		edges.sort(sort_y);
	}
	// for (uint16_t y = y_start; y < y_bound; y += 3) {
	// 	for (uint16_t x = x_start; x < x_bound; x += 3) {
	// 		if (cal_sobel(x, y) > sobel_value) {
	// 			if (x < temp.left_x
	// 					&& ((mode != 2 || x > temp.left_x - error1)
	// 							|| temp.left_x == 1000))
	// 				temp.left_x = x;
	// 			else if (x > temp.right_x
	// 					&& ((mode != 2 || x < temp.right_x + error1)
	// 							|| temp.right_x == 0))
	// 				temp.right_x = x;
	// 			if (y < temp.upper_y
	// 					&& ((mode != 2 || y > temp.upper_y - error1)
	// 							|| temp.upper_y == 1000))
	// 				temp.upper_y = y;
	// 			else if (y > temp.lower_y
	// 					&& ((mode != 2 || y < temp.lower_y + error1)
	// 							|| temp.lower_y == 0))
	// 				temp.lower_y = y;
	// 			temp.count++;
	// 		}
	// 	}
	// }
	if (edges.size() < 20)
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

//bool check_skip(uint16_t x, uint16_t y) {
//	if (ir_target == NULL)
//		return false;
//	uint8_t error = 0;
//	if (x < ir_target->right_x + error && y < ir_target->lower_y + error
//			&& y
//					> (ir_target->upper_y - error < 0 ?
//							0 : ir_target->upper_y - error)
//			&& x
//					> (ir_target->left_x - error < 0 ?
//							0 : ir_target->left_x - error))
//		return true;
//	return false;
//}

void display(Beacon temp, uint16_t color) {
	lcd->SetRegion(
			Lcd::Rect(temp.left_x, temp.upper_y, temp.right_x - temp.left_x,
					temp.lower_y - temp.upper_y));
	lcd->FillColor(color);
}

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
					insert(temp, ptr_mode::ir_Target);
					irState = seen;
					return true;
				}
			}
		}
	}
	return false;
}

bool check_near(Beacon* b1, Beacon* b2) {
	if (b1 == NULL || b2 == NULL)
		return false;
	return abs(b1->center.first - b2->center.first) < near_dist
			&& abs(b1->center.second - b2->center.second) < near_dist;
}

inline bool check_within(std::pair<uint16_t, uint16_t> center) {
	uint8_t error = 20;
	if (center.first < avoid_region.left_x - 20
			|| center.first > avoid_region.right_x + 20)
		return false;
	return true;
}

void process() {
	Beacon temp;
	if (timer && System::Time() - low_time > 100) {
		if (ir_record != NULL) {
			delete ir_record;
			ir_record = NULL;
		}
		irState = no;
		timer = false;
	}

	if (ir_record != NULL) {
		temp = check_beacon_ir(ir_record->center.first,
				ir_record->center.second);
		for (int i = 0; i < 2; i++) {
			if (temp.area > min_area) {	// 0 for ir scan, 1 for edge scan
				if (i == 0) {
					timer = false;
					if (irState == flash)
						irState = comfirm;
				} else {
					if (irState == seen)
						irState = flash;
					if (!timer) {
						low_time = System::Time();
						timer = true;
					}
				}
				insert(temp, ptr_mode::ir_Target);
				if (o_record != NULL) {		//check o_record
					temp = check_beacon_edge(1);
					if (temp.area
							> min_area /* && check_within(temp.center)*/) {
						insert(temp, ptr_mode::o_Target);
						return;		//ir and o both find -> return
					}
				}
				temp = check_beacon_edge(2);	//check the avoid region
				if (temp.area > min_area) {
					insert(temp, ptr_mode::o_Target);
					return;			//ir and o both find -> return
				}
				break;
			}
			temp = check_beacon_edge(0);
		}
		timer = false;
		delete ir_record;
		ir_record = NULL;
	}
	if (o_record != NULL) {		//ir_record must equal to NULL at this point
		temp = check_beacon_edge(1);
		if (temp.area > min_area /*&& check_within(temp.center)*/)
			insert(temp, ptr_mode::o_Target);	//only o is find -> continue
		else {
			delete o_record;
			o_record = NULL;
		}
	} else {
		temp = check_beacon_edge(2);	//check the avoid region
		if (temp.area > min_area) {
			insert(temp, ptr_mode::o_Target);
			temp = check_beacon_ir(o_target->center.first,
					o_target->center.second);//check if that beacon is also a target or not
			if (temp.area > min_area) {
				insert(temp, ptr_mode::ir_Target);
				return;
			}
		}
	}
	if (loop(0) || loop(1) || loop(2))
		return;
}
#endif /* INC_IMAGE_PROCESSING_H_ */
