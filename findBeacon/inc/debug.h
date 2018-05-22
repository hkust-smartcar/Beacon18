/*
 * debug.h
 *
 *  Created on: May 3, 2018
 *      Author: Sheldon
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_


//				if (beacon_count) {
//					lcd.SetRegion(Lcd::Rect(target->left_x, 0, 1, height));
//					lcd.FillColor(lcd.kGreen);
//					lcd.SetRegion(Lcd::Rect(target->right_x, 0, 1, height));
//					lcd.FillColor(lcd.kGreen);
//					lcd.SetRegion(Lcd::Rect(0, target->upper_y, width, 1));
//					lcd.FillColor(lcd.kGreen);
//					lcd.SetRegion(Lcd::Rect(0, target->lower_y, width, 1));
//					lcd.FillColor(lcd.kGreen);
//					lcd.SetRegion(
//							Lcd::Rect(target->center.first,
//									target->center.second, 1, 1));
//					lcd.FillColor(lcd.kBlue);
//				}
//				end = System::Time();
//				char data[10] = { };
//				sprintf(data, "%d", end - start);
//				lcd.SetRegion(Lcd::Rect(0, 0, 80, 15));
//				writer.WriteBuffer(data, 10);
//				lcd.SetRegion(Lcd::Rect(0, 105, 80, 15));
//				sprintf(data, "%d", target_count);
//				writer.WriteBuffer(data, 10);

// else if (beacon_count) {		//have possible beacon but not met requirement
//					/////record the beacon object with the highest density////
//					center_record[frame_count] = Beacon(beacons[0]);
//					for (int i = 1; i < beacon_count; i++) {
//						if (beacons[i].density
//								> center_record[frame_count].density)
//							center_record[frame_count] = Beacon(beacons[i]);
//					}
//					frame_count++;
//					/////if 5 consecutive frames have possible beacon but not sure///////
//					if (frame_count == 5) {
//						int temp = 0;
//						for (int i = 0; i < frame_count - 1; i++) {
//							if (check_near(center_record[i],
//									center_record[i + 1]))
//								temp++;
//						}
//						if (check_near(center_record[frame_count - 1],
//								center_record[0]))
//							temp++;
//						if (temp >= 3) {
//							target = center_record + (frame_count - 1);
//							frame_count = 0;
//						}
//					}
//					if (frame_count == 10) {
//						frame_count = 0;
//					}
//				}


#endif /* INC_DEBUG_H_ */
