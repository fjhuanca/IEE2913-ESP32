#pragma once

#include "constants.h"
#include <TFT_eSPI.h>
#include "BMP.h"



   // Hardware-specific library

const int VSYNC = 18; //vertical sync
const int SIOD = 21; //SDA
const int SIOC = 22; //SCL

const int RRST = 17;  //read reset
const int WRST = 16;  //write reset
const int RCK = 4;    //read clock
const int WR = 0;     //write flag
//OE -> GND     (output enable always on since we control the read clock)
//PWDN not nonnected  
//HREF not connected
//STR not connected
//RST -> 3.3V 

const int D0 = 13;
const int D1 = 12;
const int D2 = 14;
const int D3 = 27;
const int D4 = 26;
const int D5 = 25;
const int D6 = 34;
const int D7 = 39;

//const int TFT_CS = 2; //chip select
//const int TFT_RST = 0; //connected to EN on LOLIN32
//const int TFT_DC = 15; //data/command pin
//DIN <- MOSI 23
//CLK <- SCK 18
#define QQVGA
//#define QQQVGA

#ifdef QQVGA
const int XRES = 160;
const int YRES = 120;
#endif
#ifdef QQQVGA
const int XRES = 80;
const int YRES = 60;
#endif

const int BYTES_PER_PIXEL = 2;
const int frameSize = XRES * YRES * BYTES_PER_PIXEL;
// void displayRGB565(TFT_eSPI tft, unsigned char (*framep)[frameSize]);

