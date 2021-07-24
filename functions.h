#pragma once
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "constants.h"
#include <iostream>
#include <string>


// Print something in the mini status bar
void status(const char *msg, TFT_eSPI *tft);
void drawKeypad(TFT_eSPI *tft, KeyPadData *kpd);
void update_keypad(TFT_eSPI *tft, KeyPadData *kpd, uint16_t *t_x, uint16_t *t_y, bool *pressed);

void drawMenu(TFT_eSPI *tft, MenuData *md);
void update_menu(TFT_eSPI *tft, MenuData *md, uint16_t *t_x, uint16_t *t_y, bool *pressed);
void write_message(const char *msg, TFT_eSPI *tft, uint8_t x, uint8_t y);
void keypad_status(const char *msg, TFT_eSPI *tft);

void drawRegMenu(TFT_eSPI *tft, MenuRegData *md);
void update_regmenu(TFT_eSPI *tft, MenuRegData *md, uint16_t *t_x, uint16_t *t_y, bool *pressed);


void drawExitMenu(TFT_eSPI *tft, ExitMenuData *md);
void update_exitmenu(TFT_eSPI *tft, ExitMenuData *md, uint16_t *t_x, uint16_t *t_y, bool *pressed);