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
#include "helper.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;

uint32_t low_time = 0;
uint32_t high_time = 0;
bool low_timer = false;
bool high_timer = false;
uint32_t not_find_time = 0;
uint32_t full_screen_check = 0;
const int8_t y_mask[3][3] = { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
const int8_t x_mask[3][3] = { { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };
Beacon avoid_region_left(0, 25, 25, 120);	//left
Beacon avoid_region_right(165, 189, 25, 120);	//right
Beacon avoid_region_up(0, 189, 0, 25);	//up
Beacon avoid_region4(avoid_region_left.left_x, avoid_region_left.right_x,
		avoid_region_up.upper_y, avoid_region_up.lower_y + 20);
Beacon avoid_region5(avoid_region_right.left_x, avoid_region_right.right_x,
		avoid_region_up.upper_y, avoid_region_up.lower_y + 20);
Beacon no_scan(65, 130, 80, 120);	//car head
const uint16_t max_size = 5000;
const uint8_t size = 3;
const uint8_t white = 200;
const uint8_t min_area = 10;
const uint16_t near_dist = 20;
const uint8_t min_edge = 5;

inline void show_avoid_region() {
//	int len = avoid_region3.right_x - avoid_region3.left_x;
//	lcd->SetRegion(
//			Lcd::Rect(avoid_region2.right_x, avoid_region1.lower_y,
//					cam->GetW() - len, 1));
//	lcd->FillColor(Lcd::kRed);
//	lcd->SetRegion(
//			Lcd::Rect(avoid_region2.right_x, avoid_region1.lower_y, 1,
//					cam->GetH() - avoid_region1.lower_y));
//	lcd->FillColor(Lcd::kRed);
//	lcd->SetRegion(
//			Lcd::Rect(avoid_region3.left_x, avoid_region1.lower_y, 1,
//					cam->GetH() - avoid_region1.lower_y));
//	lcd->FillColor(Lcd::kRed);

//	#car head
//	int len = no_scan.right_x - no_scan.left_x;
//	lcd->SetRegion(Lcd::Rect(no_scan.left_x, no_scan.upper_y, len, 1));
//	lcd->FillColor(Lcd::kRed);
//	len = no_scan.lower_y - no_scan.upper_y;
//	lcd->SetRegion(Lcd::Rect(no_scan.left_x, no_scan.upper_y, 1, len));
//	lcd->FillColor(Lcd::kRed);
//	lcd->SetRegion(Lcd::Rect(no_scan.right_x, no_scan.upper_y, 1, len));
//	lcd->FillColor(Lcd::kRed);

}

void timer_switch(bool s) {
	bool* on_timer = NULL;
	uint32_t* on_time = NULL;
	bool* off_timer = NULL;
	if (s) {
		on_timer = &high_timer;
		on_time = &high_time;
		off_timer = &low_timer;
	} else {
		on_timer = &low_timer;
		on_time = &low_time;
		off_timer = &high_timer;
	}
	*off_timer = false;
	if (*on_timer != true) {
		*on_time = System::Time();
		*on_timer = true;
	}
}

uint16_t cal_sobel(int16_t x, int16_t y) {
	int16_t x_mean = 0;
	int16_t y_mean = 0;
	y -= 1;	//move to the upper corner
	x -= 1;
	if (x < 0 || x + 2 >= width || y < 0 || y + 2 >= height)
		return 0;
	for (uint8_t i = 0; i < 3; i++)
		for (uint8_t z = 0; z < 3; z++) {
			uint8_t pixel = buf[(y + i) * width + x + z];
			y_mean += pixel * y_mask[i][z];
			x_mean += pixel * x_mask[i][z];
		}
	return abs(x_mean) + abs(y_mean);
}

inline uint16_t cal_mean(uint16_t x, uint16_t y) {
	uint16_t mean = 0;
	uint8_t count = 0;
	for (uint8_t i = y; i < y + size && i < height; i++)
		for (uint8_t z = x; z < x + size && z < width; z++) {
			mean += buf[i * width + z];
			count += 1;
		}
	return mean / count;
}

bool sort_x(const point& first, const point& second) {
	return first.x < second.x;
}

bool sort_y(const point& first, const point& second) {
	return first.y < second.y;
}

inline bool check_near(const point p1, const point p2) {
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)) < near_dist;
}

inline bool check_valid(std::list<point> edges, std::list<point>::iterator it) {
	uint8_t count = 0;
	for (auto it2 = edges.begin(); it2 != edges.end(); ++it2)
		if (check_near(*it, *it2) && it != it2)
			if (count++ == min_edge)
				return true;
	return false;
}

inline bool check_valid2(std::list<point> edges,
		std::list<point>::reverse_iterator it) {
	uint8_t count = 0;
	for (auto it2 = edges.rbegin(); it2 != edges.rend(); ++it2)
		if (check_near(*it, *it2) && it != it2)
			if (count++ == min_edge)
				return true;
	return false;
}

void check_skip_area(uint16_t &y, uint16_t &y_bound, uint16_t &x_start,
		uint16_t &x_bound, uint16_t i) {
	switch (i) {
	case 0:
		y = 0;
		y_bound = no_scan.upper_y;
		x_start = 0;
		x_bound = width;
		break;
	case 1:
		y = no_scan.upper_y;
		y_bound = height;
		x_start = 0;
		x_bound = no_scan.left_x;
		break;
	case 2:
		y = no_scan.upper_y;
		y_bound = height;
		x_start = no_scan.right_x;
		x_bound = width;
		break;
	}
}

void check_beacon_edge(Beacon& temp, scan_mode mode) {
	Beacon* ptr = NULL;
	int8_t error = 15;
	std::list<point> edges;
	switch (mode) {
	case beacon:
		ptr = ir_record;
		break;
	case check_record:
		ptr = o_record;
		break;
	case avoid1:
		ptr = &avoid_region_left;
		error = 0;
		break;
	case avoid2:
		ptr = &avoid_region_right;
		error = 0;
		break;
	case avoid3:
		ptr = &avoid_region_up;
		error = 0;
		break;
	case avoid4:	//overlay left
		ptr = &avoid_region4;
		error = 0;
		break;
	case avoid5:	//overlay right
		ptr = &avoid_region5;
		error = 0;
		break;
	default:
		break;
	}
	if (mode != full_screen) {
		int16_t x_start = ptr->left_x < error ? 0 : ptr->left_x - error;
		int16_t x_bound = ptr->right_x + error;
		int16_t y_start = ptr->upper_y < error ? 0 : ptr->upper_y - error;
		int16_t y_bound = ptr->lower_y + error;

		if (mode < 2) {
			x_bound = x_bound > width ? width : x_bound;
			y_bound = y_bound > height ? height : y_bound;
		}
		for (uint8_t y = y_start; y < y_bound; y += 3) {
			for (uint8_t x = x_start; x < x_bound; x += 3)
				if (cal_sobel(x, y) > sobel_value) {
					point p(x, y);
					edges.push_back(p);
				}
			if (edges.size() > 150)
				break;
		}
	} else {
		uint16_t x_start = 0;
		uint16_t x_bound = 0;
		uint16_t y_start = 0;
		uint16_t y_bound = 0;
		for (int i = 0; i < 3; i++) {
			check_skip_area(y_start, y_bound, x_start, x_bound, i);
			for (uint8_t y = y_start; y < y_bound; y += 3) {
				for (uint8_t x = x_start; x < x_bound; x += 3)
					if (cal_sobel(x, y) > sobel_value) {
						point p(x, y);
						edges.push_back(p);
					}
				if (edges.size() > 150)
					break;
			}
		}
	}

	edges.sort(sort_x);
	for (int a = 0; a < 2; a++) {
		for (auto it = edges.begin(); it != edges.end(); it++)
			if (check_valid(edges, it)) {
				if (a == 0)
					temp.left_x = it->x;
				else
					temp.upper_y = it->y;
				break;
			}
		for (auto rit = edges.rbegin(); rit != edges.rend(); ++rit)
			if (check_valid2(edges, rit)) {
				if (a == 0)
					temp.right_x = rit->x;
				else
					temp.lower_y = rit->y;
				break;
			}
		edges.sort(sort_y);
	}
	temp.calc();
	if (edges.size() < 5 || temp.area > max_size)
		temp.init(0, 0);
}

void check_beacon_ir(Beacon& temp, uint16_t x, uint16_t y) {
	temp.init(x, y);
	const uint8_t error = 5;
	uint16_t* ptr = NULL;
	int16_t* moving_ptr = NULL;
	int8_t action = 0;
	for (uint8_t mode = 1; mode < 5; mode++) { //1 right,2 left,3 lower, 4 upper
		int16_t x_ptr = x;
		int16_t y_ptr = y;
		uint8_t count = 0;
		uint16_t boarder;
		switch (mode) {
		case 1:
			ptr = &temp.right_x;
			moving_ptr = &x_ptr;
			boarder = width;
			action = 1;
			break;
		case 2:
			ptr = &temp.left_x;
			moving_ptr = &x_ptr;
			boarder = 1000;
			action = -1;
			break;
		case 3:
			ptr = &temp.lower_y;
			moving_ptr = &y_ptr;
			boarder = height;
			action = 1;
			break;
		case 4:
			ptr = &temp.upper_y;
			moving_ptr = &y_ptr;
			boarder = 1000;
			action = -1;
			break;
		}
		while (true) {
			*moving_ptr += action;
			if (*moving_ptr < 0 || *moving_ptr >= boarder)
				break;
			if (buf[y_ptr * width + x_ptr] > white)  //white pixel
				*ptr = *moving_ptr;
			else if (++count > error)
				break;
		}
	}
	temp.calc();
}

bool loop_full_screen() {
	Beacon temp;
	uint16_t y;
	uint16_t y_bound;
	uint16_t x_start;
	uint16_t x_bound;
	for (int i = 0; i < 3; i++) {
		check_skip_area(y, y_bound, x_start, x_bound, i);
		for (; y < y_bound; y += size)
			for (uint16_t x = x_start; x < x_bound; x += size)
				if (cal_mean(x, y) > white) {
					check_beacon_ir(temp, x, y);
					if (temp.area > min_area) {
						insert(temp, ptr_mode::ir_Target);
						irState = seen;
						timer_switch(true);
						return true;
					}
				}
	}
	return false;
}

bool check_same(Beacon t) {
	Beacon temp;
	check_beacon_ir(temp, t.center.first, t.center.second);
	if (temp.area > min_area) {
		insert(temp, ptr_mode::ir_Target);
		irState = seen;
		timer_switch(true);
		return true;
	}
	return false;
}

//bool search_line(uint &x, uint &y, dir d) {
//	int m_y = y;
//	for (int a = 0; a < 2; a++) {
//		uint error = 0;
//		int m_x = x;
//		while (error < 10) {
//			if (cal_sobel(m_x, m_y) > sobel_value) {
//				x = m_x;
//				y = m_y;
//				return true;
//			} else {
//				error++;
//				if (a)
//					m_x++;
//				else
//					m_x--;
//				if (m_x < 0 || m_x >= width)
//					break;
//			}
//		}
//	}
//	return false;
//}
//
//std::list<regression_line> find_boarder(bool start_check) {
//	std::list<regression_line> lines;
//	std::list<std::pair<int16_t, int16_t>> line;
//	uint y = height - 3;
//	uint moving_y = y;
//	bool find = false;
//	const uint8_t error = 5;
//	uint8_t error_count = 0;
//	uint x = start_check ? 80 : 2;
//	uint x_bound = width;
//	for (; x < x_bound; x++) {	//lower bound
//		if (cal_sobel(x, y) > sobel_value) {
//			line.push_back(std::make_pair(x, y));
//			lcd->SetRegion(Lcd::Rect(x, y, 1, 1));
//			lcd->FillColor(Lcd::kRed);
//			moving_y = y - 1;
//			uint m_x = x;
//			while (true) {
//				if (search_line(m_x, moving_y, dir::v)) {
//					line.push_back(std::make_pair(m_x, moving_y));
//					lcd->SetRegion(Lcd::Rect(m_x, moving_y, 1, 1));
//					lcd->FillColor(Lcd::kRed);
//					if (line.size() >= 25) {
//						find = true;
//						break;
//					}
//					moving_y--;
//				} else if (++error_count > error)
//					break;
//			}
//			if (find) {
//				regression_line output;
//				int mean = 0;
//				int y_mean = 0;
//				for (auto it = line.begin(); it != line.end(); it++) {
//					mean += (*it).first;
//					y_mean += (*it).second;
//				}
//				mean /= line.size();
//				y_mean /= line.size();
//				int Sx = 0;
//				for (auto it = line.begin(); it != line.end(); it++)
//					Sx += pow((*it).first - mean, 2);
//				int Sxy = 0;
//				for (auto it = line.begin(); it != line.end(); it++)
//					Sxy += ((*it).first - mean) * ((*it).second - y_mean);
//				output.slope = Sx == 0 ? 0 : (float) Sxy / Sx;
//				output.intercept = y_mean - output.slope * mean;
//				for (int y = moving_y; y > 0; y--) {
//					int x2 = (y - output.intercept) / output.slope;
//					lcd->SetRegion(Lcd::Rect(x2, y, 1, 1));
//					lcd->FillColor(Lcd::kRed);
//				}
//				lines.push_back(output);
//			}
//			line.clear();
//			error_count = 0;
//		}
//	}
//	return lines;
//}

void process() {
	buf = cam->LockBuffer();
//	find_boarder();
	Beacon temp;
	if ((low_timer && System::Time() - low_time > 150)
			|| (high_timer && System::Time() - high_time > 150)) {
		if (ir_record != NULL) {
			delete ir_record;
			ir_record = NULL;
		}
		irState = no;
		low_timer = false;
		high_timer = false;
	}

	if (ir_record != NULL) {
		check_beacon_ir(temp, ir_record->center.first,
				ir_record->center.second);
		for (int i = 0; i < 2; i++) {
			if (temp.area > min_area) {	// 0 for ir scan, 1 for edge scan
				if (i == 0) {
					if (irState == flash) {
						find_time = System::Time();
						irState = checked;
					}
					timer_switch(true);
				} else {
					if (irState == seen)
						irState = flash;
					timer_switch(false);
				}
				insert(temp, ptr_mode::ir_Target);
				not_find_time = 0;
				return;
			}
			check_beacon_edge(temp, scan_mode::beacon);
		}
		timer_switch(false);
	}

	if (System::Time() - full_screen_check > 500) {
		full_screen_check = 0;
		if (loop_full_screen())
			return;
	}

	if (o_record != NULL) {		//ir not find
		check_beacon_edge(temp, scan_mode::check_record);
		if (temp.area > min_area) {
			if (check_same(temp)) {
				delete o_record;
				o_record = NULL;
			} else
				insert(temp, ptr_mode::o_Target);
			not_find_time = 0;
			return;
		}
		delete o_record;
		o_record = NULL;
	}
	for (int a = avoid1; a <= avoid5; a++) {
		check_beacon_edge(temp, static_cast<scan_mode>(a));	//check the avoid region
		if (temp.area > min_area) {
			if (!check_same(temp)) {
				insert(temp, ptr_mode::o_Target);
				o_find_time = System::Time();
			}
			not_find_time = 0;
			return;
		}
	}
	if (not_find_time == 0)
		not_find_time = System::Time();
	else if (System::Time() - not_find_time > 500) {
		not_find_time = 0;
		check_beacon_edge(temp, scan_mode::full_screen);
		if (temp.area > min_area)
			if (!check_same(temp))
				insert(temp, ptr_mode::o_Target);
	}
}
#endif /* INC_IMAGE_PROCESSING_H_ */
