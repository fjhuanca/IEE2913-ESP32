#include "functions.h"
#include <stdlib.h>
#include <string.h>

void status(const char *msg, TFT_eSPI *tft) {
  tft->setTextPadding(340);
  //tft->setCursor(STATUS_X, STATUS_Y);
  tft->setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft->setTextFont(0);
  tft->setTextDatum(TC_DATUM);
  tft->setTextSize(3);
  tft->drawString(msg, STATUS_X, STATUS_Y);
}

//------------------------------------------------------------------------------------------


void drawKeypad(TFT_eSPI *tft, KeyPadData* kpd){
    // Draw keypad background
  tft->fillRect(0, 0, 240, 320, TFT_DARKGREY);

  // Draw number display area and frame
  tft->fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_BLACK);
  tft->drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);

  // Draw the keys
  for (uint8_t row = 0; row < 5; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      uint8_t b = col + row * 3;

      if (b < 3) tft->setFreeFont(LABEL1_FONT);
      else tft->setFreeFont(LABEL2_FONT);

      kpd->key[b].initButton(tft, KEY_X + col * (KEY_W + KEY_SPACING_X),
                            KEY_Y + row * (KEY_H + KEY_SPACING_Y), // x, y, w, h, outline, fill, text
                            KEY_W, KEY_H, TFT_WHITE, kpd->keyColor[b], TFT_WHITE,
                            kpd->keyLabel[b], KEY_TEXTSIZE);
      kpd->key[b].drawButton();
    }
  }
}

// ------------------------------------------------------------------------------------------

void update_keypad(TFT_eSPI *tft, KeyPadData *kpd, uint16_t *t_x, uint16_t *t_y, bool *pressed){
 for (uint8_t b = 0; b < 15; b++) {
   if (*pressed && kpd->key[b].contains(*t_x, *t_y)) {
     kpd->key[b].press(true);  // tell the button it is pressed
   } else {
     kpd->key[b].press(false);  // tell the button it is NOT pressed
   }
 }

 // Check if any kpd->key has changed state
 for (uint8_t b = 0; b < 15; b++) {

   if (b < 3) tft->setFreeFont(LABEL1_FONT);
   else tft->setFreeFont(LABEL2_FONT);

   if (kpd->key[b].justReleased()) kpd->key[b].drawButton();     // draw normal

   if (kpd->key[b].justPressed()) {
     kpd->key[b].drawButton(true);  // draw invert

     // if a numberpad button, append the relevant # to the kpd->numberBuffer
     if (b >= 3) {
       int n = atoi(reinterpret_cast<const char*>(&(kpd->keyLabel[b][0])));
       if (*(kpd->numberIndex) < NUM_LEN) {
         if ((*(kpd->numberIndex) == 0 && n <= 2)){
          kpd->numberBuffer[*(kpd->numberIndex)] = kpd->keyLabel[b][0];
          *(kpd->numberIndex) = *(kpd->numberIndex) + 1;
          kpd->numberBuffer[*(kpd->numberIndex)] = 0; // zero terminate
          }
          else if (*(kpd->numberIndex) == 1){
            n = atoi(reinterpret_cast<const char*>(&(kpd->numberBuffer[0])));
            int n2 = atoi(reinterpret_cast<const char*>(&(kpd->keyLabel[b][0])));
            if (n<2 || (n==2 && n2<=4)){
              kpd->numberBuffer[*(kpd->numberIndex)] = kpd->keyLabel[b][0];
              *(kpd->numberIndex) = *(kpd->numberIndex) + 1;
              kpd->numberBuffer[*(kpd->numberIndex)] = 0; // zero terminate
            }
          }
          else if (*(kpd->numberIndex) > 1){
            kpd->numberBuffer[*(kpd->numberIndex)] = kpd->keyLabel[b][0];
            *(kpd->numberIndex) = *(kpd->numberIndex) + 1;
            kpd->numberBuffer[*(kpd->numberIndex)] = 0; // zero terminate
          }
       }
       keypad_status("", tft); // Clear the old keypad_status
     }

     // Del button, so delete last char
     if (b == 1) {
       kpd->numberBuffer[*(kpd->numberIndex)] = 0;
       if (*(kpd->numberIndex) > 0) {
         *(kpd->numberIndex) = *(kpd->numberIndex) - 1;
         kpd->numberBuffer[*(kpd->numberIndex)] = 0;//' ';
       }
       keypad_status("", tft); // Clear the old keypad_status
     }

     if (b == 2) {
       keypad_status("Sent value to serial port", tft);
       Serial.println(kpd->numberBuffer);
     }
     // we dont really check that the text field makes sense
     // just try to call
     if (b == 0) {
       keypad_status("Value cleared", tft);
       *(kpd->numberIndex) = 0; // Reset index to 0
       kpd->numberBuffer[*(kpd->numberIndex)] = 0; // Place null in buffer
     }

     // Update the number display field
     tft->setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
     tft->setFreeFont(&FreeSans18pt7b);  // Choose a nicefont that fits box
     tft->setTextColor(DISP_TCOLOR);     // Set the font colour

     // Draw the string, the value returned is the width in pixels
     int xwidth = tft->drawString(kpd->numberBuffer, DISP_X + 4, DISP_Y + 12);

     // Now cover up the rest of the line up by drawing a black rectangle.  No flicker this way
     // but it will not work with italic or oblique fonts due to character overlap.
     tft->fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);

     delay(10); // UI debouncing
   }
 }
}


void drawMenu(TFT_eSPI *tft, MenuData* md){
  // Draw the keys
  for (uint8_t row = 0; row < 3; row++) {
    tft->setFreeFont(LABEL2_FONT);
    uint8_t b = row;

    md->key[b].initButton(tft, MKEY_X + 0 * (MKEY_W + MKEY_SPACING_X),
                          MKEY_Y + row * (MKEY_H + MKEY_SPACING_Y), // x, y, w, h, outline, fill, text
                          MKEY_W, MKEY_H, TFT_WHITE, md->keyColor[b], TFT_WHITE,
                          md->keyLabel[b], MKEY_TEXTSIZE);
    std::string str(md->keyLabel[b]);
    md->key[b].drawButton(false, str.c_str());
  }

  for (uint8_t row = 0; row < 3; row++) {
    tft->setFreeFont(LABEL2_FONT);
    uint8_t b = row + 3;

    md->key[b].initButton(tft, 480 - MKEY_X + 0 * (MKEY_W + MKEY_SPACING_X),
                          MKEY_Y + row * (MKEY_H + MKEY_SPACING_Y), // x, y, w, h, outline, fill, text
                          MKEY_W, MKEY_H, TFT_WHITE, md->keyColor[b], TFT_WHITE,
                          md->keyLabel[b], MKEY_TEXTSIZE);
    std::string str(md->keyLabel[b]);
    md->key[b].drawButton(false, str.c_str());
  }
}

// ------------------------------------------------------------------------------------------

void update_menu(TFT_eSPI *tft, MenuData *md, uint16_t *t_x, uint16_t *t_y, bool *pressed){
 for (uint8_t b = 0; b < 6; b++) {
   if (*pressed && md->key[b].contains(*t_x, *t_y)) {
     md->key[b].press(true);  // tell the button it is pressed
   } else {
     md->key[b].press(false);  // tell the button it is NOT pressed
   }
 }

 // Check if any md->key has changed state
 for (uint8_t b = 0; b < 6; b++) {

   tft->setFreeFont(LABEL2_FONT);
    std::string str(md->keyLabel[b]);
   if (md->key[b].justReleased()) md->key[b].drawButton(false, str.c_str());     // draw normal

   if (md->key[b].justPressed()) {
     md->key[b].drawButton(true, str.c_str());  // draw invert
     *(md->selection) = b;
     delay(10); // UI debouncing
   }
 }
}


void write_message(const char *msg, TFT_eSPI *tft, uint8_t x, uint8_t y) {
  tft->setTextPadding(340);
  //tft->setCursor(STATUS_X, STATUS_Y);
  tft->setTextColor(TFT_WHITE, TFT_BLACK);
  tft->setTextFont(0);
  tft->setTextDatum(TC_DATUM);
  tft->setTextSize(2);
  tft->drawString(msg, x, y);
}

void keypad_status(const char *msg, TFT_eSPI *tft) {
  tft->setTextPadding(240);
  //tft.setCursor(STATUS_X, STATUS_Y);
  tft->setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft->setTextFont(0);
  tft->setTextDatum(TC_DATUM);
  tft->setTextSize(1);
  tft->drawString(msg, 120, 65);
}


void drawRegMenu(TFT_eSPI *tft, MenuRegData *md){
   // Draw keypad background
  tft->fillRect(240, 0, 480, 320, TFT_DARKGREY);

  // Draw number display area and frame
  tft->fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_BLACK);
  tft->drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);

  // Draw the keys
  for (uint8_t row = 0; row < 4; row++) {
    tft->setFreeFont(LABEL2_FONT);
    uint8_t b = row;

    md->key[b].initButton(tft, 480 - MKEY_X + 0 * (MKEY_W + MKEY_SPACING_X),
                          MKEY_Y + row * (MKEY_H + MKEY_SPACING_Y), // x, y, w, h, outline, fill, text
                          MKEY_W, MKEY_H, TFT_WHITE, md->keyColor[b], TFT_WHITE,
                          md->keyLabel[b], MKEY_TEXTSIZE);
    std::string str(md->keyLabel[b]);
    md->key[b].drawButton(false, str.c_str());
  }

}

void update_regmenu(TFT_eSPI *tft, MenuRegData *md, uint16_t *t_x, uint16_t *t_y, bool *pressed){
  for (uint8_t b = 0; b < 4; b++) {
    if (*pressed && md->key[b].contains(*t_x, *t_y)) {
      md->key[b].press(true);  // tell the button it is pressed
    } else {
      md->key[b].press(false);  // tell the button it is NOT pressed
    }
  }

  // Check if any md->key has changed state
  for (uint8_t b = 0; b < 4; b++) {

    tft->setFreeFont(LABEL2_FONT);
    std::string str(md->keyLabel[b]);
    if (md->key[b].justReleased()) md->key[b].drawButton(false, str.c_str());     // draw normal

    if (md->key[b].justPressed()) {
      md->key[b].drawButton(true, str.c_str());  // draw invert
      *(md->selection) = b;
      delay(10); // UI debouncing
    }
  }
}
