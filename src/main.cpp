// #define DEBUG_FRAME
#include "Frame.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <time.h>
#include "Jeedom.h"

const char VERSION[] = "1.2.25"; 

#define MAC_ADDR {0x00,0x01,0x02,0x03,0x04,0x05}

// Embbedded weather page
const char HTTP_HEADWE[] PROGMEM = "<!DOCTYPE html><html><head><title>Station&nbsp;M&eacute;t&eacute;o</title></head><body bgcolor=#DEB887>";
const char HTTP_STYLWE[] PROGMEM = "<style type=\"text/css\">.tg  {border-collapse:collapse;border-spacing:0;border-color:#93a1a1;}.tg td{font-family:Arial, sans-serif;font-size:14px;padding:10px 5px;border-style:solid;border-width:1px;overflow:hidden;word-break:normal;border-color:#93a1a1;color:#002b36;background-color:#fdf6e3;}.tg th{font-family:Arial, sans-serif;font-size:14px;font-weight:normal;padding:10px 5px;border-style:solid;border-width:1px;overflow:hidden;word-break:normal;border-color:#93a1a1;color:#fdf6e3;background-color:#657b83;}.tg .tg-lqy6{text-align:right;vertical-align:top}.tg .tg-amwm{font-weight:bold;text-align:center;vertical-align:top}.tg .tg-0lax{text-align:left;vertical-align:top}</style>";

const char HTTP_WEATHE[] PROGMEM =  "<h1 style=\"text-align: center;\">Station M&eacute;t&eacute;o</h1><p>La derni&eacute;re acquisition a &eacute;t&eacute; effectu&eacute;e le %s.</p>";
const char HTTP_TBL1WE[] PROGMEM = "<table class=\"tg\"><tr><th class=\"tg-amwm\">Capteur</th><th class=\"tg-amwm\">Mesure</th><th class=\"tg-amwm\">Valeur</th><th class=\"tg-amwm\">Unit&eacute;</th><th class=\"tg-amwm\">Observation</th></tr>";
const char HTTP_TBLRWS[] PROGMEM = "<tr> <td class=\"tg-0lax\" rowspan=\"%d\">%s</td><td class=\"tg-lqy6\">%s</td><td class=\"tg-lqy6\">%s</td><td class=\"tg-0lax\">%s</td><td class=\"tg-0lax\">%s</td></tr>";
const char HTTP_TBLRWN[] PROGMEM = "<tr> <td class=\"tg-lqy6\">%s</td><td class=\"tg-lqy6\">%s</td><td class=\"tg-0lax\">%s</td><td class=\"tg-0lax\">%s</td></tr>";

// Debug macro on long USb wire we have Error: “Brownout detector was triggered” s
// so normal working is without Uart on USB   
// #define DEBUG_WEATHER
#ifdef DEBUG_WEATHER
  #define DBXM(...) Serial.print(__VA_ARGS__)
  #define DBXMF(...) Serial.printf(__VA_ARGS__)
  #define DBXMLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBXM(...)
  #define DBXMF(...)
  #define DBXMLN(...)
#endif

//! MUST BE ADAPTED TO BME680
// I2c for Bmp280 I2C addr 0x76
#define BMP280_I2CADDR 0x76
#define pinSDA  23
#define pinSCL  22
bool BMP_is_OK = false;
Adafruit_BME280 bmp;

// Detection wind direction
#define pinU1   34
#define pinU2   35
#define pinU3   32
#define pinU4   33
#define pinU5   25
#define pinU6   26
#define pinU7   27
#define pinU8   14
const int pinWindDir[8] = {pinU1,pinU2,pinU3,pinU4,pinU5,pinU6,pinU7,pinU8};
const String windDir[8] = {"N","NE","E","SE","S","SO","O","NO"};

// Wind speed V(m/s) = r * ((2*PI)/60) * N(rpm) = K * N(rpm)
#define winRmm  50 // bucket center radius
#define pinU9   12

// Rain counter 1 liter in weather = 127.323954473516 more than 1mm on 1m2
#define rainRadus 50 // mm collector radius
#define rainCollector (PI*rainRadus*rainRadus)
#define rainFactor 1000000/rainCollector //  One litre here is equal to 127.323954473516 liter/m2
#define rainCalibration 380.0 // Average of number of flipflap for one liter
#define pinU11   15
#define pinU12   5

// Serial command
int8_t cmd;
int8_t wifiLost = 0;

// Time facilities
const long gmtOffset_sec     = 3600; // For UTC +1.00 : 1 * 60 * 60 : 3600
const int daylightOffset_sec = 3600; // Use daylight savings time (3600 or 0)
struct tm timeinfo;            // time struct
const char* ntpServer        = "pool.ntp.org";

// Time HH:MM.ss
String getTime() {
  static char temp[10];
  snprintf(temp, 20, "%02d:%02d:%02d", timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec );
  return String(temp);
}

// Date as europeen format 2019/11/25 23:58:00
String rebootTime;
String getDate(){
  static char temp[20];
  snprintf(temp, 20, "%02d/%02d/%04d %02d:%02d:%02d",
          timeinfo.tm_mday, (timeinfo.tm_mon+1), (1900+timeinfo.tm_year),  timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec );
  return String(temp);
}

// Internal led
#define EspLedBlue 2
long previousMillis = 0;

// Jeedom
Jeedom jeedom("/cfJeedom.json");
bool saveConfigJeedom = false;
// Devices from virtual jeedom
const int idTemp = 1925; // Température
const int idHumi = 1926; // % humidity
const int idBaro = 1927; // Pression barometric
const int idBars = 1935; // Pression barometric
const int idAlti = 1936; // Altidute
const int idWmps = 1937; // Wind m/s
const int idWkph = 1938; // Wind km/h
const int idRmpm = 1939; // Rain mm/m2/h 
const int idRjou = 1940; // Rain intensity per jpour
const int idWdir = 1941; // Wind direction

bool onChanged = true;

#define SEND2JEEDOM(na,wc,rj,id,va) { \
   if (wc == WL_CONNECTED && rj == HTTP_CODE_OK) { \
     rj = jeedom.sendVirtual(id, va); \
     if (rj != HTTP_CODE_OK) { \
       DBXMF("%s %s Jeedom(id:%d) error (%s)  \n\r", getDate().c_str(), na, id, httpStatus(rj)); \
      } \
    } \
  }

// Force other host name and mac Addresses  // Set config or defaults
void forceMac() {
  strlcpy(config.HostName, "esp32weather",sizeof(config.HostName));
  byte new_mac[8] = MAC_ADDR;
  for (int i=0; i<6; i++)
    config.MacAddress[i] = new_mac[i];
  config.ResetWifi = false;
  strlcpy(config.LoginName, "admin",sizeof(config.LoginName));
  strlcpy(config.LoginPassword, "admin",sizeof(config.LoginPassword));
  config.UseToolsLocal = true;
  String ret = saveConfiguration(filename, config);
  DBXMLN(ret);
}

// Read wind Direction 11110111 SUD
uint8_t getWindDir(){
  uint8_t dir = 0;
  for (int i=0; i<8; i++)
    dir += ( (1<<i) * digitalRead(pinWindDir[i]));
  return dir;
}
int getWindDirFirst(){
  for (int i=0; i<8; i++)
    if ( digitalRead(pinWindDir[i]) == LOW ) return i;
  return 0;
}
String getRoseDesVents() {
  return windDir[getWindDirFirst()];
}

// Read wind speed  U10 is not installed (else double irq)
unsigned long windCounter = 0;
void IRAM_ATTR windSpeedIrq0() {
  windCounter++;
}
void initWindSpeed() {
  pinMode(pinU9, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinU9), windSpeedIrq0, FALLING);
}
char* getWindSensor() {
  static char ret[15];
  snprintf(ret, 15, "%d", digitalRead(pinU9)  );
  return ret;
}
float winNbrRpm_1 = 0;
float windNrbRpm = 0; 
bool getWindMeterPerSec() {
  bool ret = false;
  if (winNbrRpm_1 != windNrbRpm) ret=true;
  winNbrRpm_1 = windNrbRpm;
  return ret;
}

// Rain counter 1 liter in my device = 124.129164111519 more than 1mm for 1m2
unsigned long rainFlipFlop = 0;
void IRAM_ATTR rainCounterIrq0() {
  rainFlipFlop++;
} 
void IRAM_ATTR rainCounterIrq1() {
  rainFlipFlop++;
}

float windMeterPerSec = 0;
float getWindKmPerHour() {
  return 3.6 * windMeterPerSec;
}

unsigned long rainFlipFlop_xxHxo = 0;
unsigned long rainFlipFlop_ooHoo = 0;
unsigned long rainFlipFlop_day = 0;
unsigned long windCounter_xxHxo = 0;
float rainIntensityMmxm2xh;
void updateMeteo(){ // Call every minute
  float rainNbrFFph;
  // Wind computed evey minute
  windNrbRpm = (float)(windCounter - windCounter_xxHxo) / 2.0;
  windCounter_xxHxo = windCounter;
  float winK = ((float)winRmm/1000.0)*(2.0*PI);
  windMeterPerSec = windNrbRpm * winK;
  // Rain computed evey minute
  if ( timeinfo.tm_min==0 ) { // Every Hour at xxHxo
    rainNbrFFph = (float)(rainFlipFlop - rainFlipFlop_xxHxo);
    rainFlipFlop_xxHxo = rainFlipFlop;
    if (timeinfo.tm_hour==0) { // Every Day at ooHoo
      rainFlipFlop_day = rainFlipFlop - rainFlipFlop_ooHoo;
      rainFlipFlop_ooHoo = rainFlipFlop;
    }
  } else { // Every minute
    float fact =  (60.0 / (float)timeinfo.tm_min);
    rainNbrFFph = (float)(rainFlipFlop - rainFlipFlop_xxHxo) * fact;
  }
  // Granularity = 1 / 380 *  127.323954473516 = 0.335 l.m2.h
  rainIntensityMmxm2xh = (rainNbrFFph/rainCalibration * rainFactor);
}

 float rainIntensityMmxm2xj = 0;
 bool getRainMmPerSquareMeter () {
   bool ret = false;
   float r = rainFlipFlop_day/rainCalibration * rainFactor;
   if (rainIntensityMmxm2xj != r) ret = true;
   rainIntensityMmxm2xj = r;
   return ret;
 }

char* getRainSensor(){
  static char ret[15];
  snprintf(ret, 15, "%d%db", digitalRead(pinU11) , digitalRead(pinU12) );
  return ret;
}
void initRainCounter() {
  pinMode(pinU11, INPUT_PULLUP);
  pinMode(pinU12, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinU11), rainCounterIrq0, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinU12), rainCounterIrq1, FALLING);
}

// BMP180 ######################################
float temperatureBMP = -1;
float pressureBMP = -1;
float humidityBMP = -1;
float pressureSeaBMP = -1;
bool getBMP280() {
  bool ret = false;
  if (BMP_is_OK) {
    float t = bmp.readTemperature() + jeedom.config.tempoffset;
    if ( temperatureBMP != t) {
      ret = true;
      temperatureBMP = t;
    }
    float p = bmp.readPressure() / 100.0F;
    if (pressureBMP != p) {
      ret = true;
      pressureBMP = p;
    }
    float c = 0.0065*jeedom.config.altitude;
    float a = 1.0-(c/(temperatureBMP+c+273.15));
    float b = pressureBMP * pow(a,-5.257);
    // bmp.readAltitude(SEALEVELPRESSURE_HPA);
    if (pressureSeaBMP != b) {
      ret = true;
      pressureSeaBMP = b;
    }
    float sp = bmp.readHumidity();
    if (humidityBMP != sp) {
      ret = true;
      humidityBMP = sp;
    }
  }
  return ret;
}

// Meteo HTML
char* float2cptr(float f) {
  static char ret[15];
  snprintf(ret, 15, "%.2f", f);
  return ret;
}
char* long2cptr(unsigned long f) {
  static char ret[15];
  snprintf(ret, 15, "%lu", f);
  return ret;
}
char* uint2cptr(uint8_t v) {
  static char ret[10];
  int i;
  for(i=7; i>=0; i--)
    if (((v >> i ) & 1) == 1) ret[7-i] = '1';
    else ret [7-i] = '0';
  ret[8] = 'b';
  ret[9] = 0;
  return ret;
}

char* getDebug(){
  static char ret[15];
#ifdef DEBUG_WEATHER
  snprintf(ret, 15, "USB Uart:ON" );
#else
  snprintf(ret, 15, "USB Uart:OFF" );
#endif
  return ret;
}

// Sent meteo.html to client ###########################
String httpsnp;
String sentHtmlMeteo() {
  char fmt[255];
  httpsnp.clear();
  httpsnp = FPSTR(HTTP_HEADAL);
  httpsnp += FPSTR(HTTP_STYLWE);
  // Add message date & time
  snprintf(fmt, 255, (const char*)F(HTTP_WEATHE), getDate().c_str()); httpsnp += fmt; 
  // get if action
  String srvcmd = server.arg("cmd");
  String srvval = server.arg("value");
  if (srvcmd!="") {
    if (srvcmd=="tempoffset") { //  http://192.168.1.24/meteo?cmd=tempoffset&value=-5.0
     float tempoffset = srvval.toFloat();
     jeedom.config.tempoffset = tempoffset;
     snprintf(fmt, 255, "<hr/><p>parameter:%s<br>valeur:%f</p><hr/>",srvcmd.c_str(), tempoffset); httpsnp += fmt;
    }
    if (srvcmd=="altitude") { //  http://192.168.1.24/meteo?cmd=altitude&value=-455.0
     float altitude = srvval.toFloat();
     jeedom.config.altitude = altitude;
     snprintf(fmt, 255, "<hr/><p>parameter:%s<br>valeur:%f</p><hr/>",srvcmd.c_str(), altitude); httpsnp += fmt;
    }
    jeedom.saveConfigurationJeedom();
  } else {
    // table header
    httpsnp += FPSTR(HTTP_TBL1WE);
    // Capteur BPM180
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWS), 6, "<b>BPM280</b>", "Temp&eacute;rature", float2cptr(temperatureBMP), "&deg;C", "Degr&eacute; Celsius"); httpsnp += fmt; 
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Humidit&eacute;", float2cptr(humidityBMP), "%", "");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Pression", float2cptr(pressureBMP), "hPa", "Pression atmosph&eacute;rique");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Offset", float2cptr(jeedom.config.tempoffset), "&deg;C", "Detecteur");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Altitude", float2cptr(jeedom.config.altitude), "m", "Detecteur");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Pression", float2cptr(pressureSeaBMP), "hPa", "Au niveau de la mer");httpsnp += fmt;
    // Anémomètre
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWS), 6, "<b>An&eacute;mom&egrave;tre</b>", "Vent", float2cptr(windMeterPerSec), "m.sec.", "");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Vent", float2cptr(getWindKmPerHour()), "km.h", "");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Rotation", float2cptr(windNrbRpm), "r.p.m", "Rotation par minute");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Rotation", long2cptr(windCounter), "pulses", "Nombre de tour total");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Direction", uint2cptr(getWindDir()), "binaire", "Bit(7...0): NO,O,SO,S,SE,E,NE,N");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Girouette", getRoseDesVents().c_str(), "", "<pre style=\"font-size: 10px\">rose des vents\n    N\n NO /\\ NE\n O <  > E\n SO \\/ SE\n    S</pre>");httpsnp += fmt;
    // Pluviometre
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWS), 3, "<b>Pluviom&egrave;tre</b>", "Pluie", float2cptr(rainIntensityMmxm2xh), "mm.m<sup>2</sup>.h", "Simulation par minute");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "Pluie", float2cptr(rainIntensityMmxm2xj), "mm.m<sup>2</sup>.j", "Millim&egrave;tre par jour");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "FlipFlop", long2cptr(rainFlipFlop), "pulses", "Nombre de bascule");httpsnp += fmt;
    // ESP32
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWS), 4, "<b>ESP32</b>", "Version", VERSION, "Reboot", rebootTime.c_str());httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "UTC", long2cptr(xTaskGetTickCountFromISR()), "ms", getDebug());httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "MAC", WiFi.macAddress().c_str(), "", "MAC Adresse");httpsnp += fmt;
    snprintf(fmt, 255,(const char*)F(HTTP_TBLRWN), "IP", WiFi.localIP().toString().c_str(), " FreeH:", long2cptr(ESP.getFreeHeap())); httpsnp += fmt;
  }  
  // Fin fichier
  httpsnp += ("</table></div></body></html>");
  return httpsnp;
}

#ifdef DEBUG_WEATHER
void showMeteo() {
  Serial.printf("%s WS windDir=%s winSpd=%s(c:%lu) RainS=%s(c:%lu) \n\r",  getDate().c_str(), uint2cptr(getWindDir()), getWindSensor(), windCounter, getRainSensor(), rainFlipFlop );
  Serial.printf("Bmp temp = %f Celsius \n\r", temperatureBMP);
  Serial.printf("Bmp Pres = %f hPa \n\r", pressureBMP);
  Serial.printf("Bmp Humi = %f %% \n\r", humidityBMP);
  Serial.printf("Bmp Alti = %d m \n\r", HOMEALTITUDE);
  Serial.printf("Bmp Pre0 = %f hPa \n\r", pressureSeaBMP);
  Serial.printf("Wind vit = %f m/s \n\r", windMeterPerSec);
  Serial.printf("Wind vit = %f km/h \n\r", getWindKmPerHour());
  Serial.printf("Wind cnt = %lu p \n\r", windCounter);
  Serial.printf("Wind cn1 = %lu p \n\r", windCounter_xxHxo);
  Serial.printf("Wind dir = %d  \n\r", getWindDirFirst());
  Serial.printf("Wind dir = %s rose des vents \n\r", getRoseDesVents().c_str());
  Serial.printf("Rain qua = %f mm.m2.h \n\r", rainIntensityMmxm2xh);
  Serial.printf("Rain qua = %f mm.m2.j \n\r", rainIntensityMmxm2xj);
  Serial.printf("Rain cnt = %lu p \n\r", rainFlipFlop);
  Serial.printf("Rain cn1 = %lu p \n\r", rainFlipFlop_xxHxo);
  Serial.printf("%s Opt Heap:%u ... \n\r", getDate().c_str(), ESP.getFreeHeap());
}
#endif

// Test webscoket
uint32_t value;
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
	switch(type) {
		case WStype_DISCONNECTED:
			DBXMLN("[%u] Disconnected!", num);
			break;
		case WStype_CONNECTED:
    {
      IPAddress ip = webSocket.remoteIP(num);
      DBXMLN("[%u] Connected from %d.%d.%d.%d url: [%s]", num, ip[0], ip[1], ip[2], ip[3], payload);
    	String ReponseHTML = String(value);
	    webSocket.sendTXT(num, ReponseHTML);
    }
		break;
		case WStype_TEXT:
    {
      DBXMLN("[%u] get Text: %s", num, payload);
      String _payload = String((char *) &payload[0]);
      if (payload[0] == '#')
          value = (uint32_t) strtol((const char *) &payload[1], NULL, 16);   // decode sendValue
      String ReponseHTML = String(value);
      webSocket.sendTXT(num, ReponseHTML);
    }
    break;
		case WStype_BIN:
			DBXMLN("[WSc] get binary length: %u", length);
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

// I2C scanning address
void scanI2C() {
  DBXMF("Start Scanning I2C Addresses pinSDA:%d pinSCL:%d\n\r",pinSDA,pinSCL);
  uint8_t cnt=0;
  for(uint8_t i=0;i<128;i++){
    Wire.beginTransmission(i);
    uint8_t ec = Wire.endTransmission(true);
    if(ec == 0){
      if(i < 16) DBXM('0');
      DBXM(i,HEX);
      cnt++;
      if (i == BMP280_I2CADDR) BMP_is_OK = true;
    }
    else DBXM("..");
    DBXM(' ');
    if ((i&0x0f) == 0x0f) DBXMLN("");
  }
  DBXM("Scan Completed, ");
  DBXM(cnt);
  DBXMLN(" I2C Devices found.");
}

void setup() {
#ifdef DEBUG_WEATHER
  Serial.begin(115200);
  Serial.print("Version:"); Serial.println(VERSION);
#endif
  // Set pin mode  I/O Directions
  pinMode(EspLedBlue, OUTPUT);     // Led is BLUE at statup
  digitalWrite(EspLedBlue, HIGH);  // After 5 seconds blinking indicate WiFI ids OK
  // Start framework
  frame_setup();
  // Start jeedom_ok
  jeedom.setup();
  // Append meteo html 
  server.on("/meteo", [](){
    server.send(HTTP_CODE_OK, "text/html", sentHtmlMeteo());
  });
	// Init time
	configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //init and get the time
  wifiLost = 0;
  // Start
  getLocalTime(&timeinfo);
  DBXMF("%s  Running...\r\n", getDate().c_str());
  rebootTime = getDate();
  // Wire i2c
  Wire.begin(pinSDA, pinSCL);
  scanI2C();
  // Start Bmp280
  if (BMP_is_OK)  bmp.begin(BMP280_I2CADDR);
  // Set pin mode for Wind direction
  for (int i=0; i<8; i++)
    pinMode(pinWindDir[i], INPUT); 
  // Speed meter
  initWindSpeed();
  // Rain counter
  initRainCounter();
}

// Main loop -----------------------------------------------------------------
bool fbmp = false; bool fwind = false; bool frain = false;
void loop() {

  // Call frame loop
  frame_loop();

#ifdef DEBUG_WEATHER
  // Get Serial commands
	while (Serial.available() > 0) {
	  uint8_t c = (uint8_t)Serial.read();
	  if (c != 13 && c != 10 ) {
      cmd = c;
    } else {
      if (c==13) {
        if (cmd=='h') { Serial.println(); Serial.println("- Help info:\n\r r=reboot i=myip d=debug m=MAC s=saveConfig S=ScanI2C p=pause v=verbose");}
			  else if (cmd=='r') { ESP.restart(); }
        else if (cmd=='i') { Serial.printf("Heap:%u IP:%s MAC:%s \n\r",ESP.getFreeHeap(), WiFi.localIP().toString().c_str() , WiFi.macAddress().c_str()); }
        else if (cmd=='d') { Serial.println("Mode debug active."); }
        else if (cmd=='m') { Serial.println("Mode config feilds (Mac, Host,...)."); forceMac(); cmd=' ';}
        else if (cmd=='S') { Serial.println("Mode scanning I2C."); scanI2C(); cmd=' '; }
        else if (cmd=='s') { Serial.println("Mode save config."); jeedom.saveConfigurationJeedom(); cmd=' ';}
        else if (cmd=='p') { Serial.println("Mode Pause"); }
        else if (cmd=='v') { showMeteo(); cmd = ' ';}
        else { Serial.printf("Stop serial: %s \n\r",VERSION); }
      }
		}
  }

  if ( cmd =='p' || cmd == 'P') {
    if (cmd == 'p' ) Serial.println("MODE PAUSE.");
    cmd='P';
    return;
  }
  #endif

  // Is alive executed every 1 sec.
  if ( millis() - previousMillis > 1000L) {
    previousMillis = millis();
		getLocalTime(&timeinfo);
    digitalWrite(EspLedBlue, !digitalRead(EspLedBlue));
    int jeedomStatus = HTTP_CODE_OK;
    int wifiStatus = WL_CONNECTED;

    if ( cmd=='d' ) {
      DBXMF("%s WS windDir=%s winSpd=%s(c:%lu) RainS=%s(c:%lu) \n\r", 
            getDate().c_str(), uint2cptr(getWindDir()), getWindSensor(), windCounter, getRainSensor(), rainFlipFlop );
    }

    // Read every 120 seconds
		if ( (timeinfo.tm_min % 2==0) && timeinfo.tm_sec == 30) {
      // if wifi is down, try reconnecting every 60 seconds
      wifiStatus = WiFi.status();
      if ((wifiStatus != WL_CONNECTED) ) {
        wifiLost++;
        if (wifiLost == 2 ) {
          DBXMF("%s WiFi connection is lost. cnt:%d jeeErrCnt:%d\n\r",getDate().c_str(), wifiLost, jeedom.getErrorCounter());
          saveConfigJeedom = true;
        }
        if (wifiLost == 4) {
          if (WiFi.reconnect()) {
            DBXMF("%s WiFi reconnect OK (%d). \n\r",getDate().c_str(), wifiLost);
            wifiLost = 0;
          }
        }
      } else {
        // Test if Jeedom is Connected
        if(  jeedom.getErrorCounter()!= 0 ) {
          DBXMF("%s WiFi correct but jeedom error jeeErrCnt:%d\n\r",getDate().c_str(), jeedom.getErrorCounter());
        }
      }
	  }

    // Every 1 minutes reading sensors 
    if ( ((timeinfo.tm_min%1==0) && (timeinfo.tm_sec==0)) ) {
      // wdCounter = 0; // Reset WD
      fbmp = getBMP280();
      fwind = getWindMeterPerSec();
      frain = getRainMmPerSquareMeter();
      updateMeteo();

      // Send meteo to Jeedom
      if (fbmp || fwind || frain) {
        SEND2JEEDOM("Temperatuure", wifiStatus, jeedomStatus, idTemp, temperatureBMP);
        SEND2JEEDOM("Humidity", wifiStatus, jeedomStatus, idHumi, humidityBMP);
        SEND2JEEDOM("Pression", wifiStatus, jeedomStatus, idBaro, pressureBMP);
        SEND2JEEDOM("PressionSea", wifiStatus, jeedomStatus, idBars, pressureSeaBMP);
        SEND2JEEDOM("Altidute", wifiStatus, jeedomStatus, idAlti, jeedom.config.altitude);
     
        SEND2JEEDOM("Windms", wifiStatus, jeedomStatus, idWmps, windMeterPerSec);
        SEND2JEEDOM("Windkmh", wifiStatus, jeedomStatus, idWkph, getWindKmPerHour());
        SEND2JEEDOM("Winddir", wifiStatus, jeedomStatus, idWdir, getWindDirFirst());
 
        SEND2JEEDOM("Rainmmm2", wifiStatus, jeedomStatus, idRmpm, rainIntensityMmxm2xh);
        SEND2JEEDOM("Rainxjour", wifiStatus, jeedomStatus, idRjou, rainIntensityMmxm2xj);
      }

    } // end 1 m
  } // End second
}
