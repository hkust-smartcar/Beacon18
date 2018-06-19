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

uint32_t low_time = 0;
uint32_t high_time = 0;
bool low_timer = false;
bool high_timer = false;
uint16_t max_size = 5000;

const uint8_t size = 3;
const uint8_t white = 200;
int8_t y_mask[3][3] = { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
int8_t x_mask[3][3] = { { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };
Beacon avoid_region(80, 130, 20, 100);
uint8_t min_area = 10;
uint16_t near_dist = 15;
uint8_t min_edge = 5;

inline int16_t cal_sobel(uint16_t x, uint16_t y) {
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

inline uint8_t cal_mean(uint16_t x, uint16_t y) {
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
	int8_t error = 10;
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
	int16_t x_start = (int) ptr->left_x - error;
	int16_t x_bound = (int) ptr->right_x + error;
	int16_t y_start = (int) ptr->upper_y - error;
	int16_t y_bound = (int) ptr->lower_y + error;
	std::list<point> edges;
	auto it = edges.begin();

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
//		int16_t x_start2 = 0;
//		int16_t x_bound2 = 0;
//		int16_t y_start2 = 0;
//		int16_t y_bound2 = 0;
//		for (uint8_t r = 0; r < 2; r++) {
//			switch (r) {
//			case 0:	//upper
//				x_start2 = x_start;
//				x_bound2 = x_bound;
//				y_start2 = y_start;
//				y_bound2 = y_start + (y_bound - y_start) * 0.3f;
//				break;
//			case 1:	//lower
//				x_start2 = x_start;
//				x_bound2 = x_bound;
//				y_start2 = y_bound - (y_bound - y_start) * 0.3f;
//				y_bound2 = y_bound;
//				break;
//			case 2:	//left
//				x_start2 = x_start;
//				x_bound2 = x_start + (x_bound - x_start) * 0.3f;
//				y_start2 = y_start + (y_bound - y_start) * 0.3f;
//				y_bound2 = y_bound - (y_bound - y_start) * 0.3f;
//				break;
//			case 3:	//right
//				x_start2 = x_bound - (x_bound - x_start) * 0.3f;
//				x_bound2 = x_bound;
//				y_start2 = y_start + (y_bound - y_start) * 0.3f;
//				y_bound2 = y_bound - (y_bound - y_start) * 0.3f;
//				break;
//			}
//			lcd->SetRegion(
//					Lcd::Rect(x_start2, y_start2, x_bound2 - x_start2,
//							y_bound2 - y_start2));
//			lcd->FillColor(Lcd::kGreen);
//			for (uint8_t y = y_start2; y < y_bound2; y += 2)
//				for (uint8_t x = x_start2; x < x_bound2; x += 2)
//					if (cal_sobel(x, y) > sobel_value) {
//						point p(x, y);
//						lcd->SetRegion(Lcd::Rect(x, y, 1, 1));
//						lcd->FillColor(Lcd::kRed);
//						it = edges.begin();
//						for (; it != edges.end(); it++)
//							if (check_near(*it, p)) {
//								p.count++;
//								it->count++;
//							}
//						edges.push_back(p);
//					}
//		}
//	} else {
		for (uint16_t y = y_start; y < y_bound; y += 3)
			for (uint16_t x = x_start; x < x_bound; x += 3)
				if (cal_sobel(x, y) > sobel_value) {
					point p(x, y);
					it = edges.begin();
					for (; it != edges.end(); it++)
						if (check_near(*it, p)) {
							p.count++;
							it->count++;
						}
					edges.push_back(p);
				}
//	}
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
		auto rit = edges.rbegin();
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
	temp.calc();

	if (edges.size() < 5 || temp.area > max_size)
		temp = Beacon(0, 0);
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
	uint16_t x_start;
	uint16_t x_bound;
	uint16_t y;
	uint16_t y_bound;
	switch (mode) {
	case 0:
		x_start = 0;
		x_bound = width;
		y = 0;
		y_bound = avoid_region.upper_y;
		break;
	case 1:
		x_start = 0;
		x_bound = avoid_region.left_x;
		y = avoid_region.upper_y;
		y_bound = height;
		break;
	case 2:
		x_start = avoid_region.right_x;
		x_bound = width;
		y = avoid_region.upper_y;
		y_bound = height;
		break;
	}

	for (; y < y_bound; y += size) {
		for (uint16_t x = x_start; x < x_bound; x += size) {
			if (cal_mean(x, y) > white) {
				temp = check_beacon_ir(x, y);
				if (temp.area > min_area) {
					insert(temp, ptr_mode::ir_Target);
					irState = seen;
					low_timer = false;
					return true;
				}
			}
		}
	}
	return false;
}

// bool check_near(Beacon* b1, Beacon* b2) {
// 	if (b1 == NULL || b2 == NULL)
// 		return false;
// 	return abs(b1->center.first - b2->center.first) < near_dist
// 			&& abs(b1->center.second - b2->center.second) < near_dist;
// }

bool check_same(Beacon t) {
	Beacon temp = check_beacon_ir(t.center.first, t.center.second);
	if (temp.area > min_area) {
		insert(temp, ptr_mode::ir_Target);
		irState = seen;
		low_timer = false;
		if (!high_timer) {
			high_time = System::Time();
			high_timer = true;
		}
		return true;
	}
	return false;
}

enum dir {
	h = 0, v
};

bool search_line(uint &x, uint &y, dir d) {
	int m_y = y;
	for (int a = 0; a < 2; a++) {
		uint error = 0;
		int m_x = x;
		while (error < 10) {
			if (cal_sobel(m_x, m_y) > sobel_value) {
				x = m_x;
				y = m_y;
				return true;
			} else {
				error++;
				if (a)
					m_x++;
				else
					m_x--;
				if (m_x < 0 || m_x >= width)
					break;
			}
		}
	}
	return false;
}

std::list<regression_line> find_boarder(bool start_check) {
	std::list<regression_line> lines;
	std::list<std::pair<int16_t, int16_t>> line;
	uint y = height - 3;
	uint moving_y = y;
	bool find = false;
	const uint8_t error = 5;
	uint8_t error_count = 0;
	uint x = start_check ? 80 : 2;
	uint x_bound = width;
	for (; x < x_bound; x++) {	//lower bound
		if (cal_sobel(x, y) > sobel_value) {
			line.push_back(std::make_pair(x, y));
			lcd->SetRegion(Lcd::Rect(x, y, 1, 1));
			lcd->FillColor(Lcd::kRed);
			moving_y = y - 1;
			uint m_x = x;
			while (true) {
				if (search_line(m_x, moving_y, dir::v)) {
					line.push_back(std::make_pair(m_x, moving_y));
					lcd->SetRegion(Lcd::Rect(m_x, moving_y, 1, 1));
					lcd->FillColor(Lcd::kRed);
					if (line.size() >= 25) {
						find = true;
						break;
					}
					moving_y--;
				} else if (++error_count > error)
					break;
			}
			if (find) {
				regression_line output;
				int mean = 0;
				int y_mean = 0;
				for (auto it = line.begin(); it != line.end(); it++) {
					mean += (*it).first;
					y_mean += (*it).second;
				}
				mean /= line.size();
				y_mean /= line.size();
				int Sx = 0;
				for (auto it = line.begin(); it != line.end(); it++)
					Sx += pow((*it).first - mean, 2);
				int Sxy = 0;
				for (auto it = line.begin(); it != line.end(); it++)
					Sxy += ((*it).first - mean) * ((*it).second - y_mean);
				output.slope = Sx == 0 ? 0 : (float) Sxy / Sx;
				output.intercept = y_mean - output.slope * mean;
				for (int y = moving_y; y > 0; y--) {
					int x2 = (y - output.intercept) / output.slope;
					lcd->SetRegion(Lcd::Rect(x2, y, 1, 1));
					lcd->FillColor(Lcd::kRed);
				}
				lines.push_back(output);
			}
			line.clear();
			error_count = 0;
		}
	}
	return lines;
}

void process() {

//	find_boarder();
	Beacon temp;
	if ((low_timer && System::Time() - low_time > 500)
			|| (high_timer && System::Time() - high_time > 500)) {
		if (ir_record != NULL) {
			delete ir_record;
			ir_record = NULL;
		}
		irState = no;
		low_timer = false;
		high_timer = false;
	}

	if (ir_record != NULL) {
		temp = check_beacon_ir(ir_record->center.first,
				ir_record->center.second);
		for (int i = 0; i < 2; i++) {
			if (temp.area > min_area) {	// 0 for ir scan, 1 for edge scan
				if (i == 0) {
					if (irState == flash)
						irState = checked;
					low_timer = false;
					if (!high_timer) {
						high_time = System::Time();
						high_timer = true;
					}
				} else {
					if (irState == seen)
						irState = flash;
					high_timer = false;
					if (!low_timer) {
						low_time = System::Time();
						low_timer = true;
					}
				}
				insert(temp, ptr_mode::ir_Target);
				return;
			}
			temp = check_beacon_edge(0);
		}
		if (!low_timer) {
			low_time = System::Time();
			low_timer = true;
		}
	}
	if (o_record != NULL) {		//ir not find
		temp = check_beacon_edge(1);
		if (temp.area > min_area) {
			if (check_same(temp)) {
				delete o_record;
				o_record = NULL;
			} else
				insert(temp, ptr_mode::o_Target);
			return;
		}
		delete o_record;
		o_record = NULL;
	}
	temp = check_beacon_edge(2);	//check the avoid region
	if (temp.area > min_area) {
		if (!check_same(temp))
			insert(temp, ptr_mode::o_Target);
		return;
	}
	if (loop(0) || loop(1) || loop(2))
		return;
}
#endif /* INC_IMAGE_PROCESSING_H_ */
