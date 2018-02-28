/*
 * camerafilter.h
 *
 *  Created on: 2018¦~2¤ë1¤é
 *      Author: User
 */

#ifndef INC_CAMERAFILTER_H_
#define INC_CAMERAFILTER_H_
#include "libbase/misc_utils_c.h"
const int camwidth =80;

struct Coor{
	int x;
	int y;
};
Coor findandshowpoint(bool data[][camwidth],int width,int height,Coor & previouspt){
	int w =width;
	int h =height;
	int topp;bool topf=false;
	int bottomp;bool bottomf=false;
	int leftp;bool leftf=false;
	int rightp;bool rightf=false;
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
			if(data[y][x]==1&&data[y][x+1]==0&&leftf==true){
				if(leftp<x+1){
					leftp=x+1;
				}
			}
			if(data[y][x]==0&&data[y][x+1]==1&&rightf==true){
				if(rightp>x){
					rightp=x;
				}
			}
		}
		if(topf==false&&(leftf==true||rightf==true)){
			topp=y;
			topf=true;
		}
		if(bottomf==false&&(leftf==true||rightf==true)&&topf==true){
			for(int x=0;x<w;x++){
				if(data[y][x]==0){
					allblack=false;
				}
			}
			if(allblack==true){
				bottomp=y-1;
				bottomf=true;
			}
			allblack=true;
		}

	}
	Coor centerpt;
	centerpt.x =(leftp+rightp)/2;
	centerpt.y =(topp+bottomp)/2;
	if(topf==false||bottomf==false||leftf==false||rightf==false){
				centerpt.x=previouspt.x;
				centerpt.y=previouspt.y;
			}else{
				centerpt.x =(leftp+rightp)/2;
				centerpt.y =(topp+bottomp)/2;
				previouspt.x=centerpt.x;
				previouspt.y=centerpt.y;
			}
	return centerpt;
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



#endif /* INC_CAMERAFILTER_H_ */
