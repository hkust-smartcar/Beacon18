/*
 * image_processing.h
 *
 *  Created on: May 3, 2018
 *      Author: Sheldon
 */

#ifndef INC_IMAGE_PROCESSING_H_
#define INC_IMAGE_PROCESSING_H_

// #include "libsc/lcd_typewriter.h"
#include "libsc/st7735r.h"

using namespace libsc::k60 ;

uint8_t mean(const Byte* buf, uint16_t x,uint16_t y,const uint16_t width){
	uint8_t mean = 0;
	for(uint16_t i = y ; i < y + 3 ; i++)
		mean += buf[i * width + x] +  buf[i * width + x + 1] + buf[i * width + x + 2]
	return mean;
}

void process(const Byte* buf,const uint16_t width, const uint16_t height,St7735r &lcd) {
	for(uint16_t y = 0; y < height, y++){
		for(uint16_t x = 0 ; x < width, x++){
			if(buf[y * width + x] > 225){
				
			}
		}
	}
}
#endif /* INC_IMAGE_PROCESSING_H_ */
