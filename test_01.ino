
// The SPIFFS (FLASH filing system) is used to hold touch screen
// calibration data

#include "FS.h"

#include <SPI.h>
#include <TFT_eSPI.h>      // Hardware-specific library
#include "functions.h"
#include "constants.h"
#include "calibration.h"
#include "plots.h"


#include "FifoCamera.h"
#include "I2C.h"
#include "ov7670_functions.h"
#include "wifi_data.h"
#include <WiFi.h>
#include "defines.h"
#include <WebSockets2_Generic.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>

using namespace websockets2_generic;

WebsocketsClient vital_signs_client;
WebsocketsClient ecg_client;
WebsocketsClient cam_client;



TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

// Create 15 keys for the keypad
char* keypadLabel[15][5] = {"New", "Del", "", "1", "2", "3", "4", "5", "6", "7", "8", "9", "", "0", ":" };
uint16_t keypadColor[15] = {TFT_RED, TFT_DARKGREEN, TFT_BLACK,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLACK, TFT_BLUE, TFT_BLUE
                        };


TFT_eSPI_Button keypadkey[15];

char* menuregLabel[4][15] = {"Medicamento", "Defecacion", "Comida", "Salir"};
uint16_t menuregColor[15] = {TFT_ORANGE, TFT_BROWN, TFT_DARKGREEN, TFT_RED};

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button menuregkey[6];

char* menuLabel[6][20] = {"Conectar", "Nota Voz", "Monitorear", "Test Pupila", "Mensajes", "Registros"};
uint16_t menuColor[6] = {TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE};
TFT_eSPI_Button menukey[6];

//------------------------------------------------------------------------------------------

unsigned char frame[frameSize];

I2C<SIOD, SIOC> i2c;
FifoCamera<I2C<SIOD, SIOC>, RRST, WRST, RCK, WR, D0, D1, D2, D3, D4, D5, D6, D7> camera(i2c);


//------------------------------------------------------------------------------
void setup() {

  MenuData md;
  md.key = menukey;
  md.keyLabel = *menuLabel;
  md.keyColor = menuColor;

  // Use serial port
  Serial.begin(9600);

  // Initialise the TFT screen
  tft.init();

  // Set the rotation before we calibrate
  tft.setRotation(1);

  // Calibrate the touch screen and retrieve the scaling factors
  touch_calibrate(&tft);
  // Clear the screen
  tft.fillScreen(TFT_WHITE);

  // Draw keypad
  // drawKeypad(&tft, &kpd);
  drawMenu(&tft, &md);

}

//------------------------------------------------------------------------------------------
char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;
uint8_t selection = -1;
uint8_t menureg_selection = -1;
bool redraw_menu = false;

bool connected = false;
bool cn1 = false;
bool cn2 = false;
bool cn3 = false;

void loop(void) {
  MenuData md;
  md.selection = &selection;
  md.key = menukey;
  md.keyLabel = *menuLabel;
  md.keyColor = menuColor;
  if (redraw_menu){
    // Clear the screen
    tft.fillScreen(TFT_WHITE);

    // Draw keypad
    // drawKeypad(&tft, &kpd);
    drawMenu(&tft, &md);
    redraw_menu = false;
  }
  // KeyPadData kpd;
  // kpd.numberBuffer = numberBuffer;
  // kpd.numberIndex = &numberIndex;
  // kpd.key = keypadkey;
  // kpd.keyLabel = *keypadLabel;
  // kpd.keyColor = keypadColor;  
  uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
  // // Pressed will be set true is there is a valid touch on the screen
  boolean pressed = tft.getTouch(&t_x, &t_y);
  // // / Check if any key coordinate boxes contain the touch coordinates
  // update_keypad(&tft, &kpd, &t_x, &t_y, &pressed);
  update_menu(&tft, &md, &t_x, &t_y, &pressed);
  
  if (selection==0){
    tft.fillScreen(TFT_WHITE);
    tft.setTextSize(0.6);
    // WiFi.mode(WIFI_STA);
    delay(200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      status("Conectando", &tft);
      delay(10);
      status("Conectando.", &tft);
      delay(10);
      status("Conectando..", &tft);
      delay(10);
      status("Conectando...", &tft);
    }
    status("WiFi Conectado", &tft);
    status("", &tft);
    status(IpAddress2String(WiFi.localIP()).c_str(), &tft);
    delay(1000);
    status("Conectando Servidor...", &tft);
    cn1 = vital_signs_client.connect("wss://iee2913-g10-project.southcentralus.cloudapp.azure.com/wss/receiver/signs/");
    cn2 = ecg_client.connect("wss://iee2913-g10-project.southcentralus.cloudapp.azure.com/wss/receiver/ecg/");
    tft.fillScreen(TFT_WHITE);
    if (cn1 && cn2){
      status("Servidor Conectado", &tft);
      delay(1000);
      selection = -1;
      redraw_menu = true;
    }    
  }
  else if (selection == 40){
    double ox = -999, oy = -999; // Force them to be off screen
    boolean display1 = true;
    boolean update1 = true;
    double x, y;

    tft.fillScreen(TFT_WHITE);
    tft.setTextSize(0.6);
    Graph(tft, x, y, 1, 60, 290, 390, 260, 0, 6.5, 1, -1, 1, .25, "", "", "", display1, YELLOW);
    for (x = 0; x <= 6.3; x += .1) {
      y = sin(x);
      Trace(tft, x, y, 1, 60, 290, 390, 260, 0, 6.5, 1, -1, 1, .25, "Sin(x)", "x", "fn(x)", update1, YELLOW, &ox, &oy);
      delay(50);
    }
  }
  else if (selection == 3){
      cn3 = cam_client.connect("wss://iee2913-g10-project.southcentralus.cloudapp.azure.com/wss/receiver/cam/");
      if (cn1 && cn2){
        status("Camara Conectada", &tft);
        delay(1000);
        selection = -1;
        redraw_menu = true;
      }
      tft.fillScreen(TFT_WHITE);
      i2c.init();
      camera.init();
      
      #ifdef QQVGA
        camera.QQVGARGB565();
      #endif
      #ifdef QQQVGA
        camera.QQQVGARGB565();
      #endif
      
      //camera.QQVGAYUV();
      //camera.RGBRaw();
      //camera.testImage();
      
      pinMode(VSYNC, INPUT);
      //Serial.println("start");
      tft.init();
      tft.fillScreen(TFT_WHITE);
    while (true){
      // cam_client.send(frame);
      while(!digitalRead(VSYNC));
      while(digitalRead(VSYNC));
      camera.prepareCapture();
      camera.startCapture();
      while(!digitalRead(VSYNC));
      camera.stopCapture();
      
      //color
      
      while(digitalRead(VSYNC));
      camera.readFrame(frame, XRES, YRES, BYTES_PER_PIXEL);
      displayRGB565(tft, &frame);
      // cam_client.send(reinterpret_cast<const char*>(frame));
      status("Sent", &tft);
      delay(1000);
      tft.fillScreen(TFT_WHITE);

    }
  }

  else if (selection == 2 && cn1 && cn2){
    
    tft.fillScreen(TFT_WHITE);
    while(true){
      int random1 = random(0, 10);
      char vital_data[100];
      sprintf(vital_data, "{\"temp\": %d, \"bf\": %d, \"spo\": %d, \"bpm\": %d}", random1, random1, random1, random1);
      char ecg_data[60];
      sprintf(ecg_data, "{\"value\": %d}", random1);

      vital_signs_client.send(vital_data);
      ecg_client.send(ecg_data); 
      delay(50);
    }
  }

  else if (selection == 4){
    HTTPClient msjs_client;
    tft.fillScreen(TFT_WHITE);
    status("Recolectando Msjes.", &tft);
    msjs_client.begin("https://iee2913-g10-project.southcentralus.cloudapp.azure.com/comm/messages/");
    int httpCode = msjs_client.GET();
    if (httpCode >= 200){
      String payload = msjs_client.getString();
      tft.fillScreen(TFT_WHITE);
      JSONVar myObject = JSON.parse(payload);
      JSONVar keys = myObject.keys();
      int n = myObject["n"];
      status("Ultimos 10 mensajes:", &tft);

      int msj_x = STATUS_X;
      int msj_y = STATUS_Y + 20;
      for (int i = n; i>n-10 ; i--) {
        JSONVar value = myObject[keys[i]];
        const char* msj = (const char*) value;
        write_message(msj, &tft, msj_x, msj_y);
        msj_y = msj_y + 20;
      }
    }
    delay(1000);
  }
  else if (selection == 5){
    HTTPClient register_client;
    tft.fillScreen(TFT_WHITE);
    KeyPadData kpd;
    kpd.numberBuffer = numberBuffer;
    kpd.numberIndex = &numberIndex;
    kpd.key = keypadkey;
    kpd.keyLabel = *keypadLabel;
    kpd.keyColor = keypadColor;  

    // Draw keypad
    drawKeypad(&tft, &kpd);
    // drawMenu(&tft, &md);

    MenuRegData mrpd;
    mrpd.selection = &menureg_selection;
    mrpd.key = menuregkey;
    mrpd.keyLabel = *menuregLabel;
    mrpd.keyColor = menuregColor;  

    // Draw keypad
    drawRegMenu(&tft, &mrpd);
    // drawMenu(&tft, &md);
    while (true){
      pressed = tft.getTouch(&t_x, &t_y);
      update_regmenu(&tft, &mrpd, &t_x, &t_y, &pressed);
      update_keypad(&tft, &kpd, &t_x, &t_y, &pressed);
      if (menureg_selection == 3){
        selection = -1;
        redraw_menu = true;
        menureg_selection = -1;
        break;
      }
      else if (menureg_selection == 0){
        register_client.begin("https://iee2913-g10-project.southcentralus.cloudapp.azure.com/comm/drugs_record/");
        register_client.addHeader("Content-Type", "application/json");
        tft.fillScreen(TFT_WHITE);
        char json_content[60];
        sprintf(json_content, "{\"time\": \"%s\"}", numberBuffer);
        String httpRequestData = json_content;
        int httpResponseCode = register_client.POST(httpRequestData);
        String payload = register_client.getString();
        JSONVar myObject = JSON.parse(payload);
        JSONVar value = myObject["success"];
        const char* rsp = (const char*) value;
        status("asdasd", &tft);
        delay(1000);
        menureg_selection = -1;
        if (strcmp(rsp, "false") == 0){
          break;
        } 
        else{
          selection = -1;
          status("Registrado Exitosamente", &tft);
          delay(1000);
          break;
        }
      }
       else if (menureg_selection == 1){
        register_client.begin("https://iee2913-g10-project.southcentralus.cloudapp.azure.com/comm/evacuation_record/");
        register_client.addHeader("Content-Type", "application/json");
        tft.fillScreen(TFT_WHITE);
        char json_content[60];
        sprintf(json_content, "{\"time\": \"%s\"}", numberBuffer);
        String httpRequestData = json_content;
        int httpResponseCode = register_client.POST(httpRequestData);
        String payload = register_client.getString();
        JSONVar myObject = JSON.parse(payload);
        JSONVar value = myObject["success"];
        const char* rsp = (const char*) value;
        status(rsp, &tft);
        delay(1000);
        menureg_selection = -1;
        if (strcmp(rsp, "false") == 0){
          break;
        } 
        else{
          selection = -1;
          status("Registrado Exitosamente", &tft);
          delay(1000);
          break;
        }
      }
       else if (menureg_selection == 2){
        register_client.begin("https://iee2913-g10-project.southcentralus.cloudapp.azure.com/comm/food_record/");
        register_client.addHeader("Content-Type", "application/json");
        tft.fillScreen(TFT_WHITE);
        char json_content[60];
        sprintf(json_content, "{\"time\": \"%s\"}", numberBuffer);
        String httpRequestData = json_content;
        int httpResponseCode = register_client.POST(httpRequestData);
        String payload = register_client.getString();
        JSONVar myObject = JSON.parse(payload);
        JSONVar value = myObject["success"];
        const char* rsp = (const char*) value;
        status(rsp, &tft);
        delay(1000);
        menureg_selection = -1;
        if (strcmp(rsp, "false") == 0){
          break;
        } 
        else{
          selection = -1;
          status("Registrado Exitosamente", &tft);
          delay(1000);
          break;
        }

      }
    }
  }

}


//------------------------------------------------------------------------------------------
String IpAddress2String(const IPAddress& ipAddress)
{
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

// void displayRGB565()
// {
//   tft.setAddrWindow(0, 0, YRES*2, XRES*2);
//   int i = 0;
//   for(int x = 0; x < XRES; x++){    
//       for (int p1=0; p1<2; p1++){
//         for(int y = 0; y < YRES; y++){
//           i = (y * XRES + x) << 1;
//           for (int p2=0; p2<2; p2++){
//             tft.pushColor(frame[i] | (frame[i + 1] << 8));
//             //tft.pushColor(((frame[i] | (frame[i + 1] << 8)) >> 1) & 0b111101111101111); //dimming to test for tft error
//           }
//         }
//       }
//   }     
  
// }