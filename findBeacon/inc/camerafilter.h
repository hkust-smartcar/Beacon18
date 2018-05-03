/*
 * camerafilter.h
 *
 *  Created on: 2018年2月1日
 *      Author: User
 */

#ifndef INC_CAMERAFILTER_H_
#define INC_CAMERAFILTER_H_
#include "libbase/misc_utils_c.h"
#include <libsc/k60/uart_device.h>
const int camwidth =80;
using namespace libsc::k60 ;

struct Coor{
	int x;
	int y;
};
int findandshowpoint(bool data[][camwidth],int width,int height,Coor & previouspt){///////////////////for avoiding
	int w =width;
	int h =height;
	int topp=0 ;bool topf=false;
	int bottomp=0;bool bottomf=false;
	int leftp=0;bool leftf=false;
	int rightp=0;bool rightf=false;
	int midx=9999;
	bool allblack=true;
	for(int y=0;y<h;y++){
		for(int x=0;x<w;x++){
			if(data[y][x]==1&&data[y][x+1]==0&&leftf==false){
				leftp=x+1;
				leftf=true;
			}
			if(data[y][x]==0&&data[y][x+1]==1&&rightf==false){
				rightp=x;
				rightf=true;
			}
			if(leftf==true&&rightf==true){
				midx =(leftp+rightp)/2;
				return midx;
			}
//			if(data[y][x]==1&&data[y][x+1]==0&&leftf==true){
//				if(leftp<x+1){
//					leftp=x+1;
//				}
//			}
//			if(data[y][x]==0&&data[y][x+1]==1&&rightf==true){
//				if(rightp>x){
//					rightp=x;
//				}
//			}
//		}
//		if(topf==false&&(leftf==true&&rightf==true)){
//			topp=y;
//			topf=true;
//		}
//		if(bottomf==false&&(leftf==true&&rightf==true)&&topf==true){
//			for(int x=0;x<w;x++){
//				if(data[y][x]==0){
//					allblack=false;
//				}
//			}
//			if(allblack==true){
//				bottomp=y-1;
//				bottomf=true;
//			}
			//allblack=true;
		}

	}

//	Coor centerpt;
//	centerpt.x =(leftp+rightp)/2;
	//centerpt.y =(topp+bottomp)/2;
//	if(topf==false||bottomf==false||leftf==false||rightf==false){
//				centerpt.x=previouspt.x;
//				centerpt.y=previouspt.y;
//			}else{
//				centerpt.x =(leftp+rightp)/2;
//				centerpt.y =(topp+bottomp)/2;
//				previouspt.x=centerpt.x;
//				previouspt.y=centerpt.y;
//			}
//	if(allblack==true){
//		centerpt.x =9999;
//			centerpt.y =9999;
//	}
	return midx;
}
void Bytetoboolarray(const Byte oriarray[],bool outarray[],int width,int height){
	const Uint length_ = height*width;
	Uint pos = 0;
		int bit_pos = 8;
		for (Uint row_beg = 0; row_beg < length_; row_beg += width)
		{
			for (Uint x = 0; x < width; ++x)
			{
				if (--bit_pos < 0)
				{
					bit_pos = 7;
					++pos;
				}
				if (GET_BIT(oriarray[pos], bit_pos))
				{
					outarray[x+row_beg]=1;
				}else{outarray[x+row_beg]=0;}
			}
		}
}
int threepartpixel (bool inputarray[],int width,int height){
	int leftp = 0;//1
	int rightp =0;//2
	int midp=0;//3
	for(int j =0;j<height-10;j++){
	for(int i=5;i<width/3;i++){

		if(inputarray[j*width+i]==1){
			leftp++;
		}
	}
	}
	for(int j =0;j<height-15;j++){
		for(int i=width/3;i<width*2/3-10;i++){

			if(inputarray[j*width+i]==1){
				midp++;
			}
		}
		}
	for(int j =0;j<height-10;j++){
			for(int i=width*2/3;i<width-10;i++){

				if(inputarray[j*width+i]==1){
					rightp++;
				}
			}
	}
	if(leftp<=10&&rightp<=10&&midp<=10){return 0;}
	if(leftp>midp){return (rightp>leftp)?2:1;}else{
		return (rightp>midp)?2:3;
	}
}

void onedtotwod(bool oriarray[],bool outputarray[][camwidth],int width,int height){
	for(int y=0;y<height;y++){
		for(int x=0;x<width;x++){
			outputarray[y][x]=oriarray[y*width+x];
		}
	}
}
void twodtooned(bool oriarray[][camwidth],bool outarray[],int width,int height){
	for(int y=0;y<height;y++){
		for(int x=0;x<width;x++){
			outarray[y*width+x]=oriarray[y][x];
		}
	}

}


int countwhite(bool data[][camwidth],int width,int height){
	int count=0;
	for(int i=0;i<height;i++){
		for(int j=0;j<width;j++){
			if(data[i][j]==0){count++;}
		}
	}
	return count;
}

int countblackleft(bool data[][camwidth],int width,int height){
	int count=0;
	for(int i=20;i<height;i++){
		for(int j=0;j<width/3;j++){
			if(data[i][j]==1){count++;}
		}
	}
	return count;
}
int countblackright(bool data[][camwidth],int width,int height){
	int count=0;
	for(int i=20;i<height;i++){
		for(int j=width*2/3;j<width;j++){
			if(data[i][j]==1){count++;}
		}
	}
	return count;
}
int countblackcenter(bool data[][camwidth],int width,int height){
	int count=0;
	for(int i=20;i<height;i++){
		for(int j=width*1/3;j<width*2/3;j++){
			if(data[i][j]==1){count++;}
		}
	}
	return count;
}
int countwhiteleft(bool data[][camwidth],int width,int height){
	int count=0;
	for(int i=0;i<height;i++){
		for(int j=0;j<width/3;j++){
			if(data[i][j]==0){count++;}
		}
	}
	return count;
}
int countwhiteright(bool data[][camwidth],int width,int height){
	int count=0;
	for(int i=0;i<height;i++){
		for(int j=width*2/3;j<width;j++){
			if(data[i][j]==0){count++;}
		}
	}
	return count;
}
int countwhitecenter(bool data[][camwidth],int width,int height){
	int count=0;
	for(int i=0;i<height;i++){
		for(int j=width*1/3;j<width*2/3;j++){
			if(data[i][j]==0){count++;}
		}
	}
	return count;
}



class Uartmast : public libsc::k60::UartDevice {
public:
	Uartmast(const UartDevice::Config & conf): UartDevice(UartDevice::Initializer(UartDevice::Initializer(conf))){

	}

};

//class bluetoothmast : public libsc::k60::JyMcuBt106 {
//public:
//	bluetoothmast(const UartDevice::Config & conf): UartDevice(UartDevice::Initializer(UartDevice::Initializer(conf))){
//
//	}




#endif /* INC_CAMERAFILTER_H_ */
