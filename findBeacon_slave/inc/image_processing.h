/*
 * image_processing.h
 *
 *  Created on: May 3, 2018
 *      Author: Sheldon
 */

#ifndef INC_IMAGE_PROCESSING_H_
#define INC_IMAGE_PROCESSING_H_

#include "var.h"
#include "math.h"
#include "helper.h"
#include "debug.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;

uint32_t low_time = 0;
uint32_t high_time = 0;
bool low_timer = false;
bool high_timer = false;
uint32_t full_screen_check = 0;
const int8_t y_mask[3][3] = { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
const int8_t x_mask[3][3] = { { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };

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
	if (!*on_timer) {
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

inline uint8_t check_dist(const point p1, const point p2) {
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

inline bool check_in_middle() {
	auto x = o_record->center.first;
	auto y = o_record->center.second;
//	if ((x > avoid_region_left.right_x && x < avoid_region_right.left_x
//			&& y > avoid_region_up.lower_y)
//			|| (x < avoid_region_up.right_x && x > avoid_region_up.left_x
//					&& y < avoid_region_up.lower_y))
	if (y < no_scan.upper_y && x > no_scan.left_x && x < no_scan.right_x)
		return true;
	return false;
}

std::list<point> edges;
inline bool check_valid(std::list<point>::iterator it) {
	uint8_t count = 0;
	for (auto it2 = edges.begin(); it2 != edges.end(); ++it2)
		if (check_near(*it, *it2) && it != it2)
			if (++count == min_edge)
				return true;
	return false;
}

inline bool check_valid2(std::list<point>::reverse_iterator it) {
	uint8_t count = 0;
	for (auto it2 = edges.rbegin(); it2 != edges.rend(); ++it2)
		if (check_near(*it, *it2) && it != it2)
			if (++count == min_edge)
				return true;
	return false;
}

void check_beacon_edge(Beacon& temp, scan_mode mode) {
	Beacon* ptr = NULL;
	int8_t error = 5;
	int8_t scan_size = 3;
	switch (mode) {
	case beacon:
		ptr = ir_record;
		scan_size = 2;
		break;
	case check_record:
		ptr = o_record;
		scan_size = 2;
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
			if (y_bound > no_scan.upper_y) {
				if (x_start < no_scan.right_x)
					x_start = no_scan.right_x;
				else if (x_bound > no_scan.left_x)
					x_bound = no_scan.left_x;
			}
		}
		for (uint8_t y = y_start; y < y_bound; y += scan_size) {
			for (uint8_t x = x_start; x < x_bound; x += scan_size)
				if (cal_sobel(x, y) > sobel_value) {
					point p(x, y);
					edges.push_back(p);
				}
			if (edges.size() > 150)
				break;
		}
	} else {
		uint16_t x_start = no_scan.left_x;
		uint16_t x_bound = no_scan.right_x;
		uint16_t y_start = 0;
		uint16_t y_bound = no_scan.upper_y;
		for (uint8_t y = y_start; y < y_bound; y += 3) {
			for (uint8_t x = x_start; x < x_bound; x += 3) {
				if (cal_sobel(x, y) > sobel_value) {
					point p(x, y);
					edges.push_back(p);
				}
			}
			if (edges.size() > 150)
				break;
		}
	}

	edges.sort(sort_x);
	for (int a = 0; a < 2; a++) {
		for (auto it = edges.begin(); it != edges.end(); it++)
			if (check_valid(it)) {
				if (a == 0)
					temp.left_x = it->x;
				else
					temp.upper_y = it->y;
				break;
			}
		for (auto rit = edges.rbegin(); rit != edges.rend(); ++rit)
			if (check_valid2(rit)) {
				if (a == 0)
					temp.right_x = rit->x;
				else
					temp.lower_y = rit->y;
				break;
			}
		edges.sort(sort_y);
	}
	temp.calc();
	int w = temp.right_x - temp.left_x;
	int h = temp.lower_y - temp.upper_y;
	if (edges.size() < 5 || temp.area > max_size || w > h * 3)
		temp.init(0, 0);
	edges.clear();
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

bool check_same(Beacon t) {
	Beacon temp;
	check_beacon_ir(temp, t.center.first, t.center.second);
	int diff = abs(temp.right_x - temp.left_x - (t.right_x - t.left_x));
	diff += abs(temp.lower_y - temp.upper_y - (t.lower_y - t.upper_y));
	if (diff < 30) {
		insert(temp, ptr_mode::ir_Target);
		irState = seen;
		timer_switch(true);
		return true;
	}
	return false;
}

//bool search(int &x, int &y, std::list<point>::iterator begin,
//		std::list<point>::iterator end, dir Dir) {
//	int x_move = 0;
//	int y_move = 0;
//	point *p = NULL;
//	auto it = --end;
//	int start;
//	point p_(0, 0);
//	switch (Dir) {
//	case w:
//		start = 1;
//		break;
//	case a:
//		start = 7;
//		break;
//	case s:
//		start = 5;
//		break;
//	case d:
//		start = 3;
//		break;
//	}
//	if (it != begin)
//		--it;
//	for (int i = 1; i < 4; i++) {
//		if (check_dist(point(x, y), *it) == i) {
//			p = &p_;
//			*p = *it;
//			if (it != begin)
//				--it;
//		} else
//			p = NULL;
//		int t = start;
//		int m_x;
//		int m_y;
//		while (1) {
//			if (t > 7)
//				t = 0;
//			switch (t) {
//			case 0:
//				x_move = -i;
//				y_move = 0;
//				break;
//			case 1:
//				x_move = -i;
//				y_move = -i;
//				break;
//			case 2:
//				x_move = 0;
//				y_move = -i;
//				break;
//			case 3:
//				x_move = i;
//				y_move = -i;
//				break;
//			case 4:
//				x_move = i;
//				y_move = 0;
//				break;
//			case 5:
//				x_move = i;
//				y_move = i;
//				break;
//			case 6:
//				x_move = 0;
//				y_move = i;
//				break;
//			case 7:
//				x_move = -i;
//				y_move = i;
//				break;
//			default:
//				x_move = 0;
//				y_move = 0;
//			}
//			m_x = x + x_move;
//			m_y = y + y_move;
//			if (m_x > width - 2 || m_x < 0 || m_y > height - 2 || m_y < 0)
//				;
//			else if (cal_sobel(m_x, m_y) > sobel_value) {
//				if (p != NULL && m_x == p->x && m_y == p->y) {
//
//				} else {
//					x = m_x;
//					y = m_y;
//					return true;
//				}
//			}
//			if (++t == start)
//				break;
//		}
//	}
//	return false;
//}
//
//inline bool check_near_boarder(const int x, const int y,
//		const uint8_t boarder_offset) {
//	if (x < boarder_offset || x > width - boarder_offset)
//		return true;
//	if (y < boarder_offset || y > height - boarder_offset)
//		return true;
//	return false;
//}
//
////	std::list<regression_line> lines;
//std::list<point> find_boarder() {
//	std::list<point> line_;
//	int* search_ptr = NULL;
//	int x;
//	int y;
//	int bound;
//	int act;
//	dir Dir;
//	const int8_t boarder_offset = 2;
//	for (int side = 0; side < 1; side++) { // 0 lower left, 1 lower right, 2 left,3,right, 4up
//		switch (side) {
//		case 0:
//			x = 0;
//			bound = no_scan.left_x;
//			y = height - boarder_offset;
//			search_ptr = &x;
//			act = 1;
//			Dir = w;
//			break;
//		case 1:
//			x = no_scan.right_x;
//			bound = width;
//			y = height - boarder_offset;
//			search_ptr = &x;
//			act = 1;
//			Dir = w;
//			break;
//		case 2:
//			x = boarder_offset;
//			bound = 0;
//			y = height - boarder_offset;
//			search_ptr = &y;
//			act = -1;
//			Dir = d;
//			break;
//		case 3:
//			x = width - boarder_offset;
//			bound = 0;
//			y = height - boarder_offset;
//			search_ptr = &y;
//			act = -1;
//			Dir = a;
//			break;
//		case 4:
//			x = 0;
//			bound = width;
//			y = boarder_offset;
//			search_ptr = &x;
//			act = 1;
//			Dir = s;
//			break;
//		}
//		for (; act < 0 ? *search_ptr > bound : *search_ptr < bound;
//				*search_ptr += act)
//			if (cal_sobel(x, y) > sobel_value) {	//edge find
//				line->push_back(point(x, y));
//				int moving_y = y;
//				int m_x = x;
//				while (search(m_x, moving_y, line->begin(), line->end(), Dir)) {//search for line_
//					line->push_back(point(m_x, moving_y));
//					if (m_x < 160) {
//						lcd->SetRegion(Lcd::Rect(m_x, moving_y, 1, 1));
//						lcd->FillColor(Lcd::kBlue);
//					}
//					if (line->size() > 40
//							&& check_near_boarder(m_x, moving_y, 15))
//						return line_;
//				}
//				*search_ptr += act * 10;
//				line->clear();
//			}
//	}
//	return line_;
//}

// 0 lower left, 1 lower right, 2 left,3,right, 4up
inline bool check_near_boarder(const uint8_t boarder_offset, int dir) {
//	auto first = line->begin();
//	auto last = --line->end();
	if (begin.x < boarder_offset && current.x < boarder_offset)
		return false;
	if (begin.x > width - boarder_offset && current.x > width - boarder_offset)
		return false;
	if (begin.y < boarder_offset && current.y < boarder_offset)
		return false;
	if (begin.y > height - boarder_offset
			&& current.y > height - boarder_offset)
		return false;

	if (dir != 2 && current.x < boarder_offset) {
		if (begin.x < boarder_offset)
			return false;
		return true;
	}
	if (dir != 3 && current.x > width - boarder_offset) {
		if (begin.x > width - boarder_offset)
			return false;
		return true;
	}
	if (dir != 4 && (current.y < boarder_offset)) {
		if (begin.y < boarder_offset)
			return false;
		return true;
	}
	if (((dir != 0 || dir != 1) && current.y > height - boarder_offset)) {
		if (begin.y > height - boarder_offset)
			return false;
		return true;
	}
	return false;
}

bool search_line(int &x, int &y, dir d) {
	const int error = 3;
	if (d == v) {
		int m_x = x - error;
		if (m_x < 0)
			m_x = 0;
		while (m_x < x + error && m_x < width) {
			if (cal_sobel(m_x, y) > line_sobel_value) {
				x = m_x;
				return true;
			} else
				m_x += 1;
		}
	} else {
		int m_y = y - error;
		if (m_y < 0)
			m_y = 0;
		while (m_y < y + error && m_y < height) {
			if (cal_sobel(x, m_y) > line_sobel_value) {
				y = m_y;
				return true;
			} else
				m_y += 1;
		}
	}
	return false;
}

void find_boarder() {
//	line->clear();
	int* search_ptr = NULL;
	int x;
	int y;
	int bound;
	int act;
	dir d;
	int moving_act;
	const int error = 4;
	int error_count = 0;
	const int8_t boarder_offset = 5;
	for (int a = 0; a < 5; a++) { // 0 lower left, 1 car head, 2 lower right, 3 left,4,right
		switch (a) {
		case 0:
			x = boarder_offset;
			bound = no_scan.left_x;
			y = height - boarder_offset;
			search_ptr = &x;
			act = 1;
			d = v;
			moving_act = -1;
			break;
		case 1:
			x = no_scan.left_x;
			bound = no_scan.right_x;
			y = no_scan.upper_y;
			search_ptr = &x;
			act = 1;
			d = v;
			moving_act = -1;
			break;
		case 2:
			x = no_scan.right_x;
			bound = width - boarder_offset;
			y = height - boarder_offset;
			search_ptr = &x;
			act = 1;
			d = v;
			moving_act = -1;
			break;
		case 3:
			x = boarder_offset;
			bound = 40;
			y = height - boarder_offset;
			search_ptr = &y;
			act = -1;
			d = h;
			moving_act = 1;
			break;
		case 4:
			x = width - boarder_offset;
			bound = 40;
			y = height - boarder_offset;
			search_ptr = &y;
			act = -1;
			d = h;
			moving_act = -1;
			break;
//		case 5:
//			x = boarder_offset;
//			bound = width - boarder_offset;
//			y = boarder_offset;
//			search_ptr = &x;
//			act = 1;
//			d = v;
//			moving_act = 1;
//			break;
		}
		for (; act < 0 ? *search_ptr > bound : *search_ptr < bound;
				*search_ptr += act)
			if (cal_sobel(x, y) > line_sobel_value) {	//edge find
//				line->push_back(point(x, y));
				begin.x = x;
				begin.y = y;
				int moving_y = y;
				int m_x = x;
				while (1) {
					if (search_line(m_x, moving_y, d)) {	//search for line
//						line->push_back(point(m_x, moving_y));
						current.x = m_x;
						current.y = moving_y;
						if (check_near_boarder(15, a)) {
//							print_line(line->begin(), line->end());
							end.x = m_x;
							end.y = moving_y;
							return;
						}
//							else if (line->size() > 200)
//							break;
					} else if (++error_count > error) {
						error_count = 0;
						m_x = current.x;
						moving_y = current.y;
						if (d == h) {
							d = v;
							moving_y += moving_act;
						} else {
							d = h;
							m_x += moving_act;
						}
						while (1) {
							if (search_line(m_x, moving_y, d)) {//search for line_
//								line->push_back(point(m_x, moving_y));
								current.x = m_x;
								current.y = moving_y;
								if (check_near_boarder(15, a)) {
//									print_line(line->begin(), line->end());
									end.x = m_x;
									end.y = moving_y;
									return;
								}
//									else if (line->size() > 200)
//									break;
							} else if (++error_count > error) {
								error_count = 0;
//								auto c = *--line->end();
								m_x = current.x;
								moving_y = current.y;
								if (d == h) {
									d = v;
									moving_y -= moving_act;
								} else {
									d = h;
									m_x -= moving_act;
								}
								while (1) {
									if (search_line(m_x, moving_y, d)) {//search for line_
//										line->push_back(point(m_x, moving_y));
										current.x = m_x;
										current.y = moving_y;
										if (check_near_boarder(15, a)) {
											//									print_line(line->begin(), line->end());
											end.x = m_x;
											end.y = moving_y;
											return;
										}
										//									else if (line->size() > 200)
										//									break;
									} else if (++error_count > error)
										break;
									if (d == h)
										m_x -= moving_act;
									else
										moving_y -= moving_act;
								}
								break;
							}
							if (d == h)
								m_x += moving_act;
							else
								moving_y += moving_act;
						}
						*search_ptr += act * 10;
//							line->clear();
						break;
					}
					if (d == h)
						m_x += moving_act;
					else
						moving_y += moving_act;
				}
			}
	}
	begin.x = 0;
	end.x = 0;
//	line->clear();
}

void process() {
	buf = cam->LockBuffer();
//	find_boarder();
//	if (!(begin.x == 0 && end.x == 0)) {
//		char data[20] = { };
//		lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
//		sprintf(data, "%d , %d", begin.x, begin.y);
//		writer->WriteBuffer(data, 20);
//		lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
//		sprintf(data, "%d , %d", end.x, end.y);
//		writer->WriteBuffer(data, 20);
//		lcd->SetRegion(Lcd::Rect(0, 30, 160, 15));
//		sprintf(data, "%d", (begin.y - end.y) / ( begin.x - end.x));
//		writer->WriteBuffer(data, 20);
//		return;
//	} else {
//		lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
//		writer->WriteString("No");
//	}
//	if (line->size() > 0) {
//		auto first = line->begin();
//		auto last = --line->end();
//		lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
//		char data[20] = {};
//		sprintf(data, "%d , %d", first->x, first->y);
//		writer->WriteBuffer(data, 20);
//		lcd->SetRegion(Lcd::Rect(0, 15, 160, 15));
//		sprintf(data, "%d , %d", last->x, last->y);
//		writer->WriteBuffer(data, 20);
//		return;
//	} else {
//		lcd->SetRegion(Lcd::Rect(0, 0, 160, 15));
//		writer->WriteString("No");
//		line->clear();
//	}
	Beacon temp;
	if ((low_timer && System::Time() - low_time > ir_timeout)
			|| (high_timer && System::Time() - high_time > ir_timeout)) {
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
				return;
			}
			check_beacon_edge(temp, scan_mode::beacon);
		}
		timer_switch(false);
	}

	if (System::Time() - full_screen_check > 250 && !check_in_middle()) {
		full_screen_check = System::Time();
		check_beacon_edge(temp, scan_mode::full_screen);
		if (temp.area > min_area) {
			if (!check_same(temp))
				insert(temp, ptr_mode::o_Target);
			o_find_time = System::Time();
			return;
		}
	}

	if (o_record != NULL) {		//ir not find
		check_beacon_edge(temp, scan_mode::check_record);
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
	for (int a = avoid1; a < avoid3; a++) {
		check_beacon_edge(temp, static_cast<scan_mode>(a));	//check the avoid region
		if (temp.area > min_area) {
			if (!check_same(temp)) {
				insert(temp, ptr_mode::o_Target);
				o_find_time = System::Time();
			}
			return;
		}
	}

}
#endif /* INC_IMAGE_PROCESSING_H_ */
