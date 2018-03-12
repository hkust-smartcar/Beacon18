/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libbase/k60/adc.h>
#include <libbase/k60/adc_utils.h>
#include <libsc/st7735r.h>
#include <libsc/lcd_typewriter.h>
//#include "libsc/st7735r.h"
#include <libsc/battery_meter.h>
#include <libsc/alternate_motor.h>
#include <libsc/servo.h>
#include <libsc/k60/uart_device.h>
#include <libsc/futaba_s3010.h>
#include <libsc/k60/ov7725.h>
#include <stdio.h>
//#include <cmath>
#include "../inc/camerafilter.h"
//#include "libsc/ab_encoder.h"
namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 150000;
			return config;
		}

	}
}

using libsc::System;
using namespace libsc;
using namespace libbase::k60;



Coor previoustp;


//
//bool UARTListener(const Byte *data, const size_t size) { // for RX
//
//return true;
//}



int main(void){


///////////////////////////////////////////////////////lcd
	System::Init();

	AlternateMotor::Config motorConfig;
	motorConfig.id = 0;
	motorConfig.multiplier = 100;
	AlternateMotor almotor(motorConfig);

	AlternateMotor::Config motorConfig1;
	motorConfig1.id = 1;
	motorConfig1.multiplier = 100;
	AlternateMotor armotor(motorConfig1);

	//armotor.SetClockwise(false);
	//almotor.SetClockwise(true);


//	k60::UartDevice::Config ConfigUART;
//	ConfigUART.id = 0;
//	ConfigUART.baud_rate = Uart::Config::BaudRate::k115200;
//	ConfigUART.rx_isr = UARTListener;
//	//K60::UartDevice::Initializer UART{ConfigUART};
//	k60::UartDevice uart(k60::UartDevice::Initializer(&ConfigUART));

	St7735r::Config Configlcd;
	Configlcd.fps=60;
	Configlcd.is_bgr=false;
	Configlcd.orientation=1;
	St7735r lcd(Configlcd);

	LcdTypewriter::Config writeConfig;
	writeConfig.lcd = &lcd;
	writeConfig.text_color = 0xFFFF;
	writeConfig.bg_color = 0;
	writeConfig.is_text_wrap = true;
	writeConfig.is_clear_line = true;

	 LcdTypewriter writer(writeConfig);

	k60::Ov7725::Config camconfig;
	camconfig.id =0;
	camconfig.w=80;
	camconfig.h =60;
	//camconfig.brightness=0x0;
	camconfig.contrast=0x29;
	camconfig.fps= k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 camera(camconfig);
	camera.Start();


	const int camwidth=80;
	const int camheight=60;


//	AlternateMotor::Config motorConfig;
//	motorConfig.id = id;
//	motorConfig.multiplier = 100;

	while(true){

	if(System::Time()%25==1){
	const Byte* buf = camera.LockBuffer();
	//lcd.SetRegion(Lcd::Rect(0,0,80,60));
	//lcd.FillBits(0x0000,0xFFFF,buf,60*80);
	///////////////////////////////////
		bool boolbuf[camheight*camwidth];
		Bytetoboolarray(buf,boolbuf,camwidth,camheight);

//		lcd.SetRegion(Lcd::Rect(0,60,80,60));
		//bool * buff[4800]=&boolbuf;
//		lcd.FillBits(0x0000,0xFFFF,boolbuf,60*80);
	/////////////////////////////////1dto2d
		bool twodbuf[camheight][camwidth];
		onedtotwod(boolbuf,twodbuf,camwidth,camheight);


//////////////////////////////////////alg......
//		int w =80;
//		int h =60;
//		int topp;bool topf=false;
//		int bottomp;bool bottomf=false;
//		int leftp;bool leftf=false;
//		int rightp;bool rightf=false;
//		bool allblack=true;
//		for(int y=0;y<h;y++){
//			for(int x=0;x<w;x++){
//				if(twodbuf[y][x]==1&&twodbuf[y][x+1]==0&&leftf==false){
//					leftp=x+1;
//					leftf=true;
//				}
//				if(twodbuf[y][x]==0&&twodbuf[y][x+1]==1&&rightf==false){
//					rightp=x;
//					rightf=true;
//				}
//				if(twodbuf[y][x]==1&&twodbuf[y][x+1]==0&&leftf==true){
//					if(leftp<x+1){
//						leftp=x+1;
//					}
//				}
//				if(twodbuf[y][x]==0&&twodbuf[y][x+1]==1&&rightf==true){
//					if(rightp>x){
//						rightp=x;
//					}
//				}
//			}
//			if(topf==false&&(leftf==true||rightf==true)){
//				topp=y;
//				topf=true;
//			}
//			if(bottomf==false&&(leftf==true||rightf==true)&&topf==true){
//				for(int x=0;x<w;x++){
//					if(twodbuf[y][x]==0){
//						allblack=false;
//					}
//				}
//				if(allblack==true){
//					bottomp=y-1;
//					bottomf=true;
//				}
//				allblack=true;
//			}
//
//		}
//		lcd.SetRegion(Lcd::Rect((leftp+rightp)/2,topp,2,2));
//		lcd.FillColor(lcd.kGreen);
//		lcd.SetRegion(Lcd::Rect((leftp+rightp)/2,bottomp,2,2));
//		lcd.FillColor(lcd.kRed);
//		lcd.SetRegion(Lcd::Rect((leftp+rightp)/2,(topp+bottomp)/2,2,2));
//		lcd.FillColor(lcd.kBlue);
//		previoustp.x =(leftp+rightp)/2;
//		previoustp.y =(topp+bottomp)/2;
		Coor tarpoint(findandshowpoint(twodbuf,camwidth,camheight,previoustp));
		int xpos=tarpoint.x;
		int ypos=tarpoint.y;
		int whitepiexl=countwhite(twodbuf,camwidth,camheight);
		int blackleft=countwhiteleft(twodbuf,camwidth,camheight);
		int blackright=countwhiteright(twodbuf,camwidth,camheight);
		int blackcenter=countwhitecenter(twodbuf,camwidth,camheight);

		lcd.SetRegion(Lcd::Rect(0,105,80,30));
		///////////////////////////////////////////////////////board 1
//		if(countwhite(twodbuf,camwidth,camheight)<=60){
//
//
//			if(xpos>45&&xpos<80){
//				armotor.SetClockwise(true);
//				almotor.SetClockwise(true);
//				almotor.SetPower(250);
//				armotor.SetPower(250);
//				writer.WriteString("right");
//			}else if (xpos>=0&&xpos<=35){
//					armotor.SetClockwise(false);
//					almotor.SetClockwise(false);
//					almotor.SetPower(250);
//					armotor.SetPower(250);
//					writer.WriteString("left");
//			}else if(xpos>=35&&xpos<=45){
//				armotor.SetClockwise(false);
//					almotor.SetClockwise(true);
//				almotor.SetPower(250);
//				armotor.SetPower(250);
//				writer.WriteString("forward");}
//		}else{
//			armotor.SetClockwise(false);
//			almotor.SetClockwise(true);
//			almotor.SetPower(0);
//
//			armotor.SetPower(0);
//			writer.WriteString("stop");}
//////////////////////////////////////////////////////////////////////////board 2
		if(blackleft>200){
		writer.WriteString("turn right");
		armotor.SetClockwise(true);
		almotor.SetClockwise(true);
		almotor.SetPower(180);
		armotor.SetPower(180);
		}else
		if(blackright>200){
			writer.WriteString("turn left");
			armotor.SetClockwise(false);
			almotor.SetClockwise(false);
			almotor.SetPower(180);
			armotor.SetPower(180);
		}else if(blackcenter>200){
			armotor.SetClockwise(false);
			almotor.SetClockwise(false);
			almotor.SetPower(180);
			armotor.SetPower(180);
			writer.WriteString("turn left");
		}else{
			armotor.SetClockwise(false);
			almotor.SetClockwise(true);
			almotor.SetPower(160);
			armotor.SetPower(160);
			writer.WriteString("turn forward");
		}
		lcd.SetRegion(Lcd::Rect(tarpoint.x,tarpoint.y,2,2));
		lcd.FillColor(lcd.kGreen);

		///////////////////////////////////////2dto1d
		bool newboolbuf[camheight*camwidth];
		twodtooned(twodbuf,newboolbuf,camwidth,camheight);

		lcd.SetRegion(Lcd::Rect(0,0,camwidth,camheight));
			lcd.FillBits(0x0000,0xFFFF,newboolbuf,camheight*camwidth);
			/////////////////////////////////////////////////
			char buffer [50];
			sprintf (buffer, "whitepiexl is %d,xpos %d", whitepiexl,xpos);
			lcd.SetRegion(Lcd::Rect(0,60,80,30));
			writer.WriteString(buffer);

			sprintf (buffer, "black l is %d,r is %d,c,is %d ", blackleft,blackright,blackcenter);
			lcd.SetRegion(Lcd::Rect(80,60,80,30));
			writer.WriteString(buffer);


		////////////////////////////
		camera.UnlockBuffer();

		//almotor.SetPower(100);




		}

 }
////////////////////////////////////////////////////////////////////try code

	return 0;
}

