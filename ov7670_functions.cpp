#include "ov7670_functions.h"

void displayRGB565(TFT_eSPI tft, unsigned char (*framep)[frameSize])
{
  tft.setAddrWindow(0, 0, YRES*2, XRES*2);
  int i = 0;
  for(int x = 0; x < XRES; x++){    
      for (int p1=0; p1<2; p1++){
        for(int y = 0; y < YRES; y++){
          i = (y * XRES + x) << 1;
          for (int p2=0; p2<2; p2++){
            tft.pushColor(*framep[i] | (*framep[i + 1] << 8));
            //tft.pushColor(((frame[i] | (frame[i + 1] << 8)) >> 1) & 0b111101111101111); //dimming to test for tft error
          }
        }
      }
  }     
  
}