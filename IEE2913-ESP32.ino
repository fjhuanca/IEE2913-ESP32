
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
// #include <WebSockets2_Generic.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <Arduino_JSON.h>
#include "BMP.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "wav.h"
#include "driver/i2s.h"
#include "i2s_handler.h"

#include <WebSocketsClient.h>
#include <Adafruit_MPU6050.h>



// -------------------- Pantalla ---------------------------------------------//

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

char* keypadLabel[15][5] = {"New", "Del", "", "1", "2", "3", "4", "5", "6", "7", "8", "9", "", "0", "" };
uint16_t keypadColor[15] = {TFT_RED, TFT_DARKGREEN, TFT_BLACK,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLACK, TFT_BLUE, TFT_BLACK
                        };


TFT_eSPI_Button keypadkey[15];

char* menuregLabel[4][15] = {"Medicamento", "Defecacion", "Comida", "Salir"};
uint16_t menuregColor[15] = {TFT_ORANGE, TFT_BROWN, TFT_DARKGREEN, TFT_RED};

TFT_eSPI_Button menuregkey[6];

char* menuLabel[6][20] = {"Conectar", "Nota Voz", "Monitorear", "Test Pupila", "Mensajes", "Registros"};
uint16_t menuColor[6] = {TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE};
TFT_eSPI_Button menukey[6];

char* exitregLabel[6][20] = {"Terminar"};
uint16_t exitregColor[6] = {TFT_RED};
TFT_eSPI_Button exitregkey[6];

char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;
uint8_t selection = -1;
uint8_t menureg_selection = -1;
uint8_t exit_selection = -1;
bool redraw_menu = false;

// -------------------- Comunicacion -----------------------------------------//
char servername[] = "iee2913-g10-project.southcentralus.cloudapp.azure.com";
String url_cam = "/wss/receiver/cam/";
String url_signs = "/wss/receiver/signs/";
String url_ecg = "/wss/receiver/ecg/";
String url_info_in = "/wss/sender/info/";
String url_messages = "/comm/messages/";
String url_drugs = "/comm/drugs_record/";
String url_evacuation = "/comm/evacuation_record/";
String url_food = "/comm/food_record/";
String url_audio = "/comm/voice_note/";
WebSocketsClient webSocket_signs;
WebSocketsClient webSocket_ecg;
WebSocketsClient webSocket_info_in;
#define TIMEOUT 5000
#define LED_PIN 25
int camera_on = false;

bool connected = false;
bool cn1 = false;
bool cn2 = false;
bool cn3 = false;

String IpAddress2String(const IPAddress& ipAddress){
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

void webSocket_info_in_event(WStype_t type, uint8_t * payload, size_t length) {
  JSONVar myObject;
  int n;
  int nm;
  int sc;
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED:
      Serial.printf("[WSc] Connected to url: %s\n", payload);

      // send message to server when Connected
      //webSocket.sendTXT("Connected");
      break;
    case WStype_TEXT:
      Serial.printf("[WSc] get text: %s\n", payload);
      myObject = JSON.parse(String((char *) payload));
      n = myObject["led"];
      nm = myObject["new_message"];
      sc = myObject["action_request"];
      Serial.printf("%d\n", n);
      if (n && camera_on) digitalWrite(LED_PIN, HIGH);
      else if (sc == 1){
        status(" Dr. solicita monitoreo ", &tft);
      }
      else if (sc == 2){
        status("Dr. solicita test pupila", &tft);
      }
      else if (nm) {
        status("      Nuevo Mensaje     ", &tft);
      }
      else if (!n) digitalWrite(LED_PIN, LOW);

      // send message to server
      // webSocket.sendTXT("message here");
      break;
    case WStype_BIN:
      Serial.printf("[WSc] get binary length: %u\n", length);
      // hexdump(payload, length);

      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
    case WStype_ERROR:      
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

// -------------------- Frecuencia Respiratoria ------------------------------//
Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
int16_t ax, ay, az, gx, gy, gz; // variables del sensor

const int numReadings = 40; //numero de muestras para promedio
int readings[numReadings];// Lecturas de la entrada analogica
int index2 = 0; //el indice de la lectura actual
int total = 0; // Total
float averageX = 0; // Promedio

int pulsos = 0;
int pulsos2 = 0;
int senal = 0;

int st_anterior = 0;
int st_actual = 0;
int respiro = 0;
long int t = 0; 
const int muestras_R = 100;
int respiros[muestras_R];
int idx_respiros = 0;

int calculo_freq_respiratoria(){
  total = total - readings[index2];
  readings[index2] = ax;
  total = total + readings[index2];
  index2 = index2 + 1 ;
  if (index2 >= numReadings) index2 = 0;
  averageX = total / numReadings;
  averageX=averageX+averageX;
  if (averageX<=70){
    averageX=50;
    st_actual = 0;
    if (st_actual != st_anterior){
        respiro = 0;
        st_anterior = 0;
    }
  }
  else {
    st_actual = 1;
    if (st_actual != st_anterior){
      respiro = 1;
      st_anterior = 1;
    }
  }
  if (millis()-t >= 200){
    t = millis();
    if (respiro == 1){
      respiros[idx_respiros] = 1;
      idx_respiros += 1;
      if (idx_respiros == muestras_R) idx_respiros = 0;
      respiro = 0;
    }
    else{
      respiros[idx_respiros] = 0;
      idx_respiros += 1;
      if (idx_respiros == muestras_R) idx_respiros = 0;
    }
  }
  pulsos = 0;
  for (int q=0; q<muestras_R; q++){
    if (respiros[q]) pulsos += 1;
  }
  pulsos2 = pulsos * 3;
  return pulsos2;
}


// -------------------- Nota de Voz ------------------------------------------//


WiFiClient audio_client;
const char filename[] = "/temp.wav";
bool recorded = false;
File file;

void SPIFFSInit(){
  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS initialisation failed!");
    while(1) yield();
  }

  SPIFFS.remove(filename);
  file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("File is not available!");
  }

  byte header[headerSize];
  wavHeader(header, FLASH_RECORD_SIZE);

  file.write(header, headerSize);
  //listSPIFFS();
}


void i2s_adc(void *arg){
    
    int i2s_read_len = I2S_READ_LEN;
    int flash_wr_size = 0;
    size_t bytes_read;

    char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));

    i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    
    Serial.println(" *** Recording Start *** ");
    while (flash_wr_size < FLASH_RECORD_SIZE) {
        //read data from I2S bus, in this case, from ADC.
        i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        //example_disp_buf((uint8_t*) i2s_read_buff, 64);
        //save original data from I2S(ADC) into flash.
        i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
        //file.write((const byte*) i2s_read_buff, i2s_read_len);
        file.write((const byte*) flash_write_buff, i2s_read_len);
        flash_wr_size += i2s_read_len;
        //ets_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
        //ets_printf("Never Used Stack Size: %u\n", uxTaskGetStackHighWaterMark(NULL));
    }
    Serial.println(" *** Recording Stop *** ");
    file.close();

    free(i2s_read_buff);
    i2s_read_buff = NULL;
    free(flash_write_buff);
    flash_write_buff = NULL;
    
    //listSPIFFS();
    recorded = true;
    // vTaskDelete(NULL);
}


bool send_wav(){
   file = SPIFFS.open(filename);
   if (audio_client.connect(servername, 8000)) {
      Serial.println("connected");
      audio_client.print(String("POST ") + url_audio + " HTTP/1.1\r\n" + "Host: " + servername + "\r\n");
      audio_client.println("User-Agent: ESP32");
      audio_client.println("Accept-Encoding: gzip, deflate");
      audio_client.println("Accept: */*");
      audio_client.println("Connection: keep-alive");
      audio_client.print("Content-Length: ");
      audio_client.println(file.size()+142);
      Serial.println(file.size());
      audio_client.println("Content-Type: multipart/form-data; boundary=da5bc2523dad070ffab70d9707be5265");
      audio_client.println("");
      audio_client.println("--da5bc2523dad070ffab70d9707be5265");
      audio_client.println("Content-Disposition: form-data; name=\"audio\"; filename=\"temp.wav\"");
      audio_client.println("");
      //audio_client.println();
      //while (file.available()){
      //      audio_client.write(file.read());
      //} 
      audio_client.write(file);
      Serial.println("");
      audio_client.println("--da5bc2523dad070ffab70d9707be5265--");
      Serial.println(">>><<<");
      if (audio_client.available()){
        String serverRes = audio_client.readStringUntil('}');
        Serial.println(serverRes);
      }
    }
  file.close();
}


// -------------------- Temperatura ------------------------------------------//
#define ONE_WIRE_BUS 32
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tempC = 0;

void get_temp(void *arg){
  sensors.requestTemperatures();
  tempC = (int)(sensors.getTempCByIndex(0) * 10) / 10.0;
  vTaskDelete(NULL);
}


// -------------------- Oximetria y Pulso ------------------------------------//
#define REPORTING_PERIOD_MS     1000
PulseOximeter pox;
uint32_t tsLastReport = 0;


// -------------------- Camara -----------------------------------------------//

unsigned char bmpHeader[BMP::headerSize];
unsigned char frame[frameSize];

I2C<SIOD, SIOC> i2c;
FifoCamera<I2C<SIOD, SIOC>, RRST, WRST, RCK, WR, D0, D1, D2, D3, D4, D5, D6, D7> camera(i2c);

void displayRGB565(unsigned char * frame, int xres, int yres){
  tft.setAddrWindow(0, 0, yres - 1, xres - 1);
  int i = 0;
  for(int x = 0; x < xres; x++)
    for(int y = 0; y < yres; y++){
      i = (y * xres + x) << 1;
      tft.pushColor(frame[i] | (frame[i + 1] << 8));
      //tft.pushColor(((frame[i] | (frame[i + 1] << 8)) >> 1) & 0b111101111101111); //dimming to test for tft error
    }  
}

void readFrame(){
  while(!digitalRead(VSYNC));
  while(digitalRead(VSYNC));
  camera.stopCapture();
  camera.prepareCapture();
  while(!digitalRead(VSYNC));
  camera.startCapture();
  camera.readFrame(frame, XRES, YRES, BYTES_PER_PIXEL);
}

// -------------------- UART ---------------------------------------------//
// #define RXD2 16
// #define TXD2 17

uint8_t read_serial(){
  if(Serial.available()>0) {
    char incomingByte = Serial.read();
    int choice = incomingByte - '0';
    Serial.println(choice);
    return (uint8_t)choice;
  }
  return -1;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  MenuData md;
  md.key = menukey;
  md.keyLabel = *menuLabel;
  md.keyColor = menuColor;
  pinMode(23, INPUT); // Setup for leads off detection LO 
  pinMode(36, INPUT); // Setup for leads off detection AD8232 

  // Use serial port
  Serial.begin(115200,SERIAL_8N1);


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

void loop(void) {
  webSocket_signs.loop();
  webSocket_ecg.loop();
  webSocket_info_in.loop();
  MenuData md;
  md.selection = &selection;
  md.key = menukey;
  md.keyLabel = *menuLabel;
  md.keyColor = menuColor;
  ExitMenuData expd;
  expd.selection = &exit_selection;
  expd.key = exitregkey;
  expd.keyLabel = *exitregLabel;
  expd.keyColor = exitregColor;
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
  if (!pressed) selection = read_serial();
  
  if (selection==0 ){
    if (WiFi.status() != WL_CONNECTED){
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
      webSocket_info_in.begin(servername, 8000, url_info_in);
      webSocket_info_in.onEvent(webSocket_info_in_event);
      webSocket_info_in.setReconnectInterval(5000);
      delay(1000);
      tft.fillScreen(TFT_WHITE);
      while (!webSocket_info_in.isConnected()){
        status("Conectando Servidor...", &tft);
        webSocket_info_in.loop();
      }
    }
    else{
      status("Ya Conectado", &tft);
      delay(1000);
    }
    selection = -1;
    redraw_menu = true;
  }

  else if (selection == 3){
    WebSocketsClient webSocket_cam;
    webSocket_cam.begin(servername, 8000, url_cam);
    webSocket_cam.setReconnectInterval(5000);
      
      // i2c.init();
      // camera.init();
      // camera.QQVGARGB565();
      // cn3 = cam_client.connect("wss://iee2913-g10-project.southcentralus.cloudapp.azure.com/wss/receiver/cam/");
      // if (cn3){
      //   status("Camara Conectada", &tft);
      //   delay(1000);
      //   selection = -1;
      //   redraw_menu = true;
      // }
      tft.fillScreen(TFT_WHITE);
      drawExitMenu(&tft, &expd);
      camera_on = true;
      while (exit_selection != 0){     
        webSocket_info_in.loop();   
        pressed = tft.getTouch(&t_x, &t_y);
        if (!pressed) exit_selection = read_serial();
        // digitalWrite(LED_PIN, 1);        
        update_exitmenu(&tft, &expd, &t_x, &t_y, &pressed);
        // readFrame();
        // displayRGB565(frame, XRES, YRES);
        // char fram2send[BMP::headerSize];
        // sprintf(fram2send, "%s", frame);
        // cam_client.send(fram2send);
      }
        camera_on = false;
        menureg_selection = -1;
        exit_selection = -1;
        selection = -1;
        redraw_menu = true;
        // digitalWrite(LED_PIN, 0);
  }

  else if (selection == 2 && WiFi.status() == WL_CONNECTED){
    webSocket_signs.begin(servername, 8000, url_signs);
    webSocket_signs.setReconnectInterval(5000);
    webSocket_ecg.begin(servername, 8000, url_ecg);
    webSocket_ecg.setReconnectInterval(5000);
    long t = millis();
    webSocket_signs.loop();
    webSocket_ecg.loop();
    while (!cn1 || !cn2 || millis() - t > TIMEOUT){
      webSocket_signs.loop();
      webSocket_ecg.loop();
      cn1 = webSocket_signs.isConnected();
      cn2 = webSocket_ecg.isConnected();
      status("Conectando Servidor...", &tft);
      t = millis();
    }

    delay(1000);
    cn1 = webSocket_signs.isConnected();
    cn2 = webSocket_ecg.isConnected();
    tft.fillScreen(TFT_WHITE);
    if (cn1 && cn2){
      status("Servidor Conectado", &tft);
      delay(1000);
      sensors.begin();
      sensors.setWaitForConversion(false);
      long tpox = millis();
      // while (millis() - tpox < 5000 && !pox.begin());
      mpu.begin();
      mpu_gyro = mpu.getGyroSensor();
      tft.fillScreen(TFT_WHITE);
      long last_millis_1 = millis();
      long last_millis_2 = last_millis_1;
      long last_millis_3 = last_millis_1;
      int spo2 = 0;
      int hr = 0;
      drawExitMenu(&tft, &expd);

      write_message("Monitoreando", &tft, STATUS_X, 160, 4);
      while(exit_selection !=0 ){
        webSocket_info_in.loop();
        pressed = tft.getTouch(&t_x, &t_y);
        update_exitmenu(&tft, &expd, &t_x, &t_y, &pressed);
        // pox.update();
        sensors_event_t gyro;
        mpu_gyro->getEvent(&gyro);
        ax = (gyro.gyro.x*100);
        int bf = calculo_freq_respiratoria();
        
        
        if (millis() - last_millis_1 >= 1000){
          
          TaskHandle_t xHandle = NULL;
          xTaskCreate(get_temp, "get_temp", 1024 * 1, NULL, 4, NULL);
          // spo2 = pox.getSpO2();
          // hr = pox.getHeartRate();
          
          char vital_data[100];
          sprintf(vital_data, "{\"temp\": %f, \"bf\": %d, \"spo\": %d, \"bpm\": %d}", tempC, bf, spo2, hr);
          webSocket_signs.sendTXT(vital_data);
          last_millis_1 = millis();
        }
        // if (millis() - last_millis_3 >= 10000){
        //   spo2 = 98;
        //   hr = 75;
        //   sensors.requestTemperatures();
        //   tempC = (int)(sensors.getTempCByIndex(0) * 10) / 10.0;
        //   char vital_data[100];
        //   sprintf(vital_data, "{\"temp\": %f, \"bf\": %d, \"spo\": %d, \"bpm\": %d}", tempC, 0, spo2, hr);
        //   vital_signs_client.send(vital_data);
        //   last_millis_3 = millis();
        // }
        if (millis() - last_millis_2 >= 5 ){
          uint16_t ecg_read = analogRead(36);
          char ecg_data[60];
          sprintf(ecg_data, "{\"value\": %d}", ecg_read);
          webSocket_ecg.sendTXT(ecg_data); 
          last_millis_2 = millis();
        }
      }
    }
    else{
      status("Fallo conexion", &tft);
      delay(1000);
    }
  webSocket_ecg.disconnect();
  webSocket_signs.disconnect();
  cn1 = false;
  cn2 = false;
  menureg_selection = -1;
  exit_selection = -1;
  selection = -1;
  redraw_menu = true;
  }

  else if (selection == 4){
    HTTPClient msjs_client;
    tft.fillScreen(TFT_WHITE);
    status("Recolectando Mensajes", &tft);
    msjs_client.begin("https://" + String(servername) + url_messages);
    int httpCode = msjs_client.GET();
    drawExitMenu(&tft, &expd);
    if (httpCode >= 200){
      String payload = msjs_client.getString();
      JSONVar myObject = JSON.parse(payload);
      JSONVar keys = myObject.keys();
      int n = myObject["n"];
      int n2 = std::min(n, 7);
      status("Ultimos 7 mensajes:", &tft);

      int msj_x = STATUS_X;
      int msj_y = STATUS_Y + 30;
      for (int i = 0; i<n2 ; i++) {
        JSONVar value = myObject[keys[n-i-1]];
        const char* msj = (const char*) value;
        write_message(msj, &tft, msj_x, msj_y);
        msj_y = msj_y + 30;
        delay(500);
      }
    }
    exit_selection = -1;
    while (exit_selection != 0){
      pressed = tft.getTouch(&t_x, &t_y);
      update_exitmenu(&tft, &expd, &t_x, &t_y, &pressed);
      if (!pressed) exit_selection = read_serial();
    }
    menureg_selection = -1;
    exit_selection = -1;
    selection = -1;
    redraw_menu = true;
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
        register_client.begin("https://" + String(servername) + url_drugs);
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
        status("Enviado", &tft);
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
        register_client.begin("https://" + String(servername) + url_evacuation);
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
        register_client.begin("https://" + String(servername) + url_food);
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
  else if (selection == 1){
      tft.fillScreen(TFT_WHITE);
      write_message("Grabando en 3...", &tft, STATUS_X, 160, 4);
      delay(1000);
      write_message("Grabando en 2...", &tft, STATUS_X, 160, 4);
      delay(1000);
      write_message("Grabando en 1...", &tft, STATUS_X, 160, 4);
      delay(1000);      
      write_message("Grabando en 0...", &tft, STATUS_X, 160, 4);
      delay(1000);
      SPIFFSInit();
      i2sInit();
      // xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, NULL);
      write_message("  Grabando ...  ", &tft, STATUS_X, 160, 4);
      i2s_adc(NULL);
      write_message("Listo, enviando ...", &tft, STATUS_X, 160, 4);
      send_wav();
      recorded = false;
      write_message("     Enviado!      ", &tft, STATUS_X, 160, 4);
      exit_selection = -1;
      selection = -1;
      redraw_menu = true;

  }

}


//------------------------------------------------------------------------------------------

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
