// #define DEBUG_FRAME
#include "Frame.h"
#include "DHT.h"

#include <Wire.h>
#include <Adafruit_BMP085.h> // A powerful but easy to use BMP085/BMP180 Library
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>

#include <time.h>
#include "Jeedom.h"

const char VERSION[] = "Ver:0.0.3";

#define MAC_ADDR {0x30,0xAE,0xA4,0x90,0xFD,0xD9}

// Embbedded weather page
const char HTTP_HEADWE[] PROGMEM = "<!DOCTYPE html><html><head><title>Station&nbsp;M&eacute;t&eacute;o</title></head><body bgcolor=#DEB887>";
const char HTTP_STYLWE[] PROGMEM = "<style type=\"text/css\">.tg  {border-collapse:collapse;border-spacing:0;border-color:#93a1a1;}.tg td{font-family:Arial, sans-serif;font-size:14px;padding:10px 5px;border-style:solid;border-width:1px;overflow:hidden;word-break:normal;border-color:#93a1a1;color:#002b36;background-color:#fdf6e3;}.tg th{font-family:Arial, sans-serif;font-size:14px;font-weight:normal;padding:10px 5px;border-style:solid;border-width:1px;overflow:hidden;word-break:normal;border-color:#93a1a1;color:#fdf6e3;background-color:#657b83;}.tg .tg-lqy6{text-align:right;vertical-align:top}.tg .tg-amwm{font-weight:bold;text-align:center;vertical-align:top}.tg .tg-0lax{text-align:left;vertical-align:top}</style>";

const char HTTP_WEATHE[] PROGMEM =  "<h1 style=\"text-align: center;\">Station M&eacute;t&eacute;o</h1><p>La derni&eacute;re acquisition a &eacute;t&eacute; effectu&eacute;e le %s.</p>";
const char HTTP_TBL1WE[] PROGMEM = "<table class=\"tg\"><tr><th class=\"tg-amwm\">Capteur</th><th class=\"tg-amwm\">Mesure</th><th class=\"tg-amwm\">Valeur</th><th class=\"tg-amwm\">Unit&eacute;</th><th class=\"tg-amwm\">Observation</th></tr>";
const char HTTP_TBLRWS[] PROGMEM = "<tr> <td class=\"tg-0lax\" rowspan=\"%d\">%s</td><td class=\"tg-lqy6\">%s</td><td class=\"tg-lqy6\">%s</td><td class=\"tg-0lax\">%s</td><td class=\"tg-0lax\">%s</td></tr>";
const char HTTP_TBLRWN[] PROGMEM = "<tr> <td class=\"tg-lqy6\">%s</td><td class=\"tg-lqy6\">%s</td><td class=\"tg-0lax\">%s</td><td class=\"tg-0lax\">%s</td></tr>";

// Debug macro
#ifdef DEBUG_MAIN
  #define DBXM(...) Serdial.print(__VA_ARGS__)
  #define DBXMLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBXM(...)
  #define DBXMLN(...)
#endif

// I2c for Bmp180 I2C addr 0x77
#define pinSDA  22
#define pinSCL  23
bool BMP_is_OK = false;
Adafruit_BMP085 bmp;
//Adafruit_BME280 bmp;

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
// r = 12.180038 mm  --> K = 0.012180038 * 2.PI/60 --> V(m/s) = 0.00127549059670815 * N(rpm)
// Wind spped V(km/h) = 3.6 * V(m/s)
#define winRmm  50 // bucket center radius
#define winK    (winRmm/1000)*(2*PI)/60
#define pinU9   12
#define pinU10  13

// Rain counter 1 liter in weather = 127.323954473516 more than 1mm on 1m2
#define rainRadus 50 // mm collector radius
#define rainCollector (PI*rainRadus*rainRadus)
#define rainFactor 1000000/rainCollector //  One litre here is equal to 127.323954473516 liter/m2
#define rainCalibration 60 // Average of number of flipflap for one liter here
#define pinU11   15
#define pinU12   5

// Serial command
int8_t cmd;
int8_t wifiLost = 0;

// DHT22 pin 16
#define pinDHT     16 // GPIO16
DHT dht(pinDHT, DHT22);

// Time facilities
const long gmtOffset_sec     = 3600;
const int daylightOffset_sec = 3600;
struct tm timeinfo;            // time struct
const char* ntpServer        = "pool.ntp.org";

// Time HH:MM.ss
String getTime() {
  char temp[10];
  snprintf(temp, 20, "%02d:%02d:%02d", timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec );
  return String(temp);
}

// Date as europeen format
String getDate(){
  char temp[20];
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
const int idTemp = 1984; // Temperature
const int idBaro = 1984; // Pression barometric
const int idHumi = 1984; // % humidity
const int idWdir = 1984; // Wind direction
const int idWmps = 1984; // Wind m/s
const int idWkph = 1984; // Wind km/h
const int idRmpm = 1984; // Rain mm/m2 or l/m2 last 5 minute
const int idRint = 1984; // Rain intensity per minute
bool onChanged = true;

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
  Serial.println(ret);
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

// Read wind speed WARNING magnet polarization inverted
// Detect 1/2 turn
unsigned long windspeed[2];
void windSpeedIrq() {
  if ( digitalRead(pinU9) == LOW && digitalRead(pinU10) == HIGH ) {
    windspeed[0] = millis();
  } else {
    if ( digitalRead(pinU9) == HIGH && digitalRead(pinU10) == LOW ) {
      windspeed[1] = millis();
    }
  }
}
void initWindSpeed() {
  pinMode(pinU9, INPUT_PULLUP);
  pinMode(pinU10, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinU9), windSpeedIrq, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinU10), windSpeedIrq, FALLING);
}
float windMeterPerSec = 0;
bool getWindMeterPerSec() {
  bool ret = false;
  unsigned long ecartMs = abs(windspeed[1] - windspeed[0]) * 2;
  float nbrRpm = 0; // rotation per minute
  if (ecartMs != 0) nbrRpm = 60000.0 / (float)ecartMs;
  if (nbrRpm * winK != windMeterPerSec) ret = true;
  windMeterPerSec = nbrRpm * winK;
  return ret;
}
float getWindKmPerHour() {
  return 3.6 * windMeterPerSec;
}

// Rain counter 1 liter in my device = 124.129164111519 more than 1mm for 1m2
unsigned long raincounter[2];
unsigned long rainFlipFlop = 0;
float rainMmPerSquareMeterPerHour = 0;
void rainCounterIrq() {
  if ( digitalRead(pinU11) == LOW && digitalRead(pinU12) == HIGH ) { // Flip
    raincounter[0] = millis();
    rainFlipFlop++;
  } else {
    if ( digitalRead(pinU11) == HIGH && digitalRead(pinU12) == LOW ) { // Flop
      raincounter[1] = millis();
      rainFlipFlop++;
    }
  }
  if (timeinfo.tm_hour==0 && timeinfo.tm_min==0 && timeinfo.tm_sec==0) {
     rainMmPerSquareMeterPerHour = rainFlipFlop/rainCalibration;
     rainFlipFlop = 0; //RESET every hour
   }
}
void initRainCounter() {
  pinMode(pinU11, INPUT_PULLUP);
  pinMode(pinU12, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinU11), rainCounterIrq, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinU12), rainCounterIrq, FALLING);
}

float rainMmPerSquareMeter = 0;
bool getRainMmPerSquareMeter () {
  bool ret = false;
  float r = rainFlipFlop/rainCalibration;
  if (rainMmPerSquareMeter != r) ret = true;
  rainMmPerSquareMeter = r;
  return ret;
}
// Rain instantly
float getRainIntensityMmPerMinute() {
  unsigned long ecartMs = abs(raincounter[1] - raincounter[0]);
  float ffpm = 0; // flipFlop per minute
  if (ecartMs != 0) ffpm = 60000.0 / (float)ecartMs;
  return ffpm / rainCalibration;
}

// DHT22 ############################
float humidityDHT = -1;
float temperatureDHT = -1;
bool getDHTHumidity(){
  bool ret = false;
  float h = dht.readHumidity();
  if (isnan(h)) return ret;
  if (humidityDHT != h) ret = true;
  humidityDHT = h;
  return ret;
}
bool getDHTTemperature(){
  bool ret = false;
  float t = dht.readTemperature();
  if (isnan(t)) {
    Serial.printf("%s DHT ERROR RET:%d \n\r", getDate().c_str(), ret);
    return ret;
  }
  if (temperatureDHT != t) ret = true;
  temperatureDHT = t;
  return ret;
}

// BMP180 ######################################
float temperatureBMP = -1;
float pressureBMP = -1;
float pressureSeaBMP = -1;
float altitudeBMP = -1;
bool getBMP180() {
  bool ret = false;
  if (BMP_is_OK) {
    float t = bmp.readTemperature();
    if ( temperatureBMP != t) {
      ret = true;
      temperatureBMP = t;
    }
    float p = bmp.readPressure();
    if (pressureBMP != p) {
      ret = true;
      pressureBMP = p;
    }
    // bmp.readAltitude(101500) with pressure a sea level
    float a = bmp.readAltitude();
    if (altitudeBMP != a) {
      ret = true;
      altitudeBMP = a;
    }
    float sp = bmp.readSealevelPressure();
    if (pressureSeaBMP != sp) {
      ret = true;
      pressureSeaBMP = sp;
    }
  }
  return ret;
}

// Meteo HTML
char* float2cptr(float f) {
  static char ret[15];
  sprintf(ret, "%.2f", f);
  return ret;
}
char* long2cptr(unsigned long f) {
  static char ret[15];
  sprintf(ret, "%lu", f);
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

// Write meteo.html file into FS ###########################
String meteoHtml() {
  File file = SPIFFS.open("/meteo.html", FILE_WRITE);
  if (!file)
    return F("Can't write in meteo.html file.");
  // Store meteo file
  file.print( FPSTR(HTTP_HEADWE));
  file.print( FPSTR(HTTP_STYLWE));
  // Add message date & time
  file.printf( FPSTR(HTTP_WEATHE), getDate().c_str());
  // table header
  file.print( FPSTR(HTTP_TBL1WE));
  // Capteur DHT22
  file.printf( FPSTR(HTTP_TBLRWS), 2, "<b>DTH22</b>", "Temp&eacute;rature", float2cptr(temperatureDHT), "&deg;C", "");
  file.printf( FPSTR(HTTP_TBLRWN), "Humidit&eacute;", float2cptr(humidityDHT), "%", "");
  // Capteur BPM180
  file.printf( FPSTR(HTTP_TBLRWS), 4, "<b>BPM180</b>", "Temp&eacute;rature", float2cptr(temperatureBMP), "&deg;C", "");
  file.printf( FPSTR(HTTP_TBLRWN), "Altitude", float2cptr(altitudeBMP), "m", "");
  file.printf( FPSTR(HTTP_TBLRWN), "Pression 1", float2cptr(pressureBMP), "hPa", "Altitude du capteur");
  file.printf( FPSTR(HTTP_TBLRWN), "Pression 2", float2cptr(pressureSeaBMP), "hPa", "Niveau de la mer");
  // Anémomètre
  file.printf( FPSTR(HTTP_TBLRWS), 6, "<b>An&eacute;mom&egrave;tre</b>", "Vent", float2cptr(windMeterPerSec), "m/sec.", "");
  file.printf( FPSTR(HTTP_TBLRWN), "Vent", float2cptr(getWindKmPerHour()), "km/h", "");
  file.printf( FPSTR(HTTP_TBLRWN), "1/2 tour", long2cptr(windspeed[0]), "ms", "1/2 tour mesure interval en milli-secondes.");
  file.printf( FPSTR(HTTP_TBLRWN), "1   tour", long2cptr(windspeed[1]), "ms", "    (50 jours max.) ");
  file.printf( FPSTR(HTTP_TBLRWN), "Direction", uint2cptr(getWindDir()), "binaire", "");
  file.printf( FPSTR(HTTP_TBLRWN), "Girouette", getRoseDesVents().c_str(), "", "<pre>rose des vents\n    N\n NO /\\ NE\n O <  > E\n SO \\/ SE\n    S</pre>");

  // Pluviometre
  file.printf( FPSTR(HTTP_TBLRWS), 6, "<b>Pluviom&egrave;tre</b>", "Pluie", float2cptr(rainMmPerSquareMeter), "mm/m<sup>2</sup>", "Rain instantly");
  file.printf( FPSTR(HTTP_TBLRWN), "Pluie", float2cptr(getRainIntensityMmPerMinute()), "mm/60s", "millim&egrave;tre par minute");
  file.printf( FPSTR(HTTP_TBLRWN), "Pluie", float2cptr(rainMmPerSquareMeterPerHour), "mm/m<sup>2</sup>/h", "Last hour");
  file.printf( FPSTR(HTTP_TBLRWN), "FlipFlop", long2cptr(rainFlipFlop), "pulse", "Number of FlipFlop");
  file.printf( FPSTR(HTTP_TBLRWN), "Godet 1", long2cptr(raincounter[0]), "ms", "Bascule godet 1");
  file.printf( FPSTR(HTTP_TBLRWN), "Godet 2", long2cptr(raincounter[1]), "ms", "Bascule godet 2");
  // ESP32
  file.printf( FPSTR(HTTP_TBLRWS), 3, "<b>ESP32</b>", "Taille heap", long2cptr(ESP.getFreeHeap()), "bytes", "");
  file.printf( FPSTR(HTTP_TBLRWN), "MAC", WiFi.macAddress().c_str(), "", "MAC Adresse");
  file.printf( FPSTR(HTTP_TBLRWN), "IP", WiFi.localIP().toString().c_str(), "IPV4", "IP adresse");

  // Fin fichier
  file.print("</table></div></body></html>");
  file.close();
  return F("File meteo.html file has been saved.");
}

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

// received argument from Jeedom script via virutal
String handleJeedom() {
  String ret ="jeedom_ok";
  String srvcmd = server.arg("cmd");
  String srvval = server.arg("value");
  if (srvcmd!="") {
//    if (srvcmd=="update" ) { // Msg cmd=update
//      if (srvval!="") jeedom.config.fluxReference = srvval.toFloat(); // SET
//      else ret = String(jeedom.config.fluxReference); // No value tag value => GET
//    }
    // From jeedom ...
    if (jeedom.isCcrChanged()) saveConfigJeedom = true;
    if (cmd=='d' || cmd=='f')
      Serial.printf("%s Jeedom srvcmd:%s srvval:%s \n\r",((srvval!="")?("Set"):("Get")), srvcmd.c_str(), srvval.c_str());
  }
  return ret;
}

// I2C csanning address
void scanI2C() {
  Serial.printf("Start Scanning I2C Addresses pinSDA:%d pinSCL:%d\n\r",pinSDA,pinSCL);
  uint8_t cnt=0;
  for(uint8_t i=0;i<128;i++){
    Wire.beginTransmission(i);
    uint8_t ec = Wire.endTransmission(true);
    if(ec == 0){
      if(i < 16) Serial.print('0');
      Serial.print(i,HEX);
      cnt++;
      if (i == BMP085_I2CADDR) BMP_is_OK = true;
    }
    else Serial.print("..");
    Serial.print(' ');
    if ((i&0x0f) == 0x0f) Serial.println();
  }
  Serial.print("Scan Completed, ");
  Serial.print(cnt);
  Serial.println(" I2C Devices found.");
}

// WatchDog:  wdCounter is set to 0 otherwise after 15 minutes ESP is restarted
uint32_t wdCounter = 0;
void watchdog(void *pvParameter) {
  while (1) {
    vTaskDelay(5000/portTICK_RATE_MS);
    wdCounter++;
    if (wdCounter > 180) ESP.restart(); // Restart after 5sec * 180 => 15min
  }
}

void setup() {
  Serial.begin(115200);
  Serial.print("Version:"); Serial.println(VERSION);
  // Start my WatchDog
  xTaskCreate(&watchdog, "wd task", 2048, NULL, 5, NULL);
  // Set pin mode  I/O Directions
  pinMode(EspLedBlue, OUTPUT);     // Led is BLUE at statup
  digitalWrite(EspLedBlue, HIGH);  // After 5 seconds blinking indicate WiFI ids OK
  // Start framework
  frame_setup();
  // Start jeedom_ok
  jeedom.setup();
  server.on("/jeedom", [](){
     server.send(HTTP_CODE_OK, "text/plain", handleJeedom());
  });
	// Init time
	configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //init and get the time
  wifiLost = 0;
  // Start
  getLocalTime(&timeinfo);
  Serial.printf("%s  Running...\r\n", getDate().c_str());

  // Wire i2c
  Wire.begin(pinSDA, pinSCL);
  scanI2C();
  // Start Bmp180
  bmp.begin();
  // Start DHT22
  dht.begin();
  // Speed meter
  initWindSpeed();
  // Rain counter
  initRainCounter();
}

// Main loop -----------------------------------------------------------------
void loop() {
	while (Serial.available() > 0) {
	  uint8_t c = (uint8_t)Serial.read();
	  if (c != 13 && c != 10 ) {
      cmd = c;
    } else {
      if (c==13) {
        if (cmd=='h') { Serial.println(); Serial.println("- Help info: r=reboot i=myip d=debug f=force m=MAC j=jeedom");}
			  else if (cmd=='r') { ESP.restart(); }
        else if (cmd=='i') { Serial.printf("Heap:%u IP:%s \n\r",ESP.getFreeHeap(), WiFi.localIP().toString().c_str() ); }
        else if (cmd=='d') { Serial.println("Mode debug active."); }
        else if (cmd=='f') { Serial.println("Mode force active."); }
        else if (cmd=='m') { Serial.println("Mode config feilds (Mac, Host,...)."); forceMac(); }
        else if (cmd=='j') { Serial.println("Mode jeeDom active."); }
        else { Serial.printf("Stop serial: %s \n\r",VERSION); }
      }
		}
  }

  // Call frame loop
  frame_loop();

  // Is alive executed every 1 sec.
  if ( millis() - previousMillis > 1000L) {
    previousMillis = millis();
		getLocalTime(&timeinfo);
    digitalWrite(EspLedBlue, !digitalRead(EspLedBlue));
    int  retJeedom = 0;

    if ( cmd=='d' ) {
      Serial.printf("%s ... Weather Station wdCounter=%d \n\r",  getDate().c_str(),  wdCounter );
    }

    // DHT read every 120 seconds
		if ( (timeinfo.tm_min % 2==0) && timeinfo.tm_sec == 30) {
      // if wifi is down, try reconnecting every 60 seconds
      if ((WiFi.status() != WL_CONNECTED) ) {
        wifiLost++;
        if (wifiLost == 2 ) {
          Serial.printf("%s WiFi connection is lost. cnt:%d jeeErrCnt:%d\n\r",getDate().c_str(), wifiLost, jeedom.getErrorCounter());
          saveConfigJeedom = true;
        }
        if (wifiLost == 4) {
          if (WiFi.reconnect()) {
            Serial.printf("%s WiFi reconnect OK (%d). \n\r",getDate().c_str(), wifiLost);
            wifiLost = 0;
          }
        }
      } else {
        // Test if Jeedom is Connected
        if(  jeedom.getErrorCounter()!= 0 ) {
          Serial.printf("%s WiFi correct but jeedom error jeeErrCnt:%d\n\r",getDate().c_str(), jeedom.getErrorCounter());
        }
      }
	  }

    // Every 5 minutes record T and H
    if ( ((timeinfo.tm_min % 5==0) && (timeinfo.tm_sec == 0)) || cmd=='f' ) {
      Serial.printf("%s - Every 5mm (cmd:%c)...", getDate().c_str(), cmd);
      wdCounter = 0; // Reset WD
      if (getDHTTemperature()) {
        Serial.print("temperatureDHT = ");
        Serial.print(temperatureDHT);
        Serial.println(" Celsius");
      }
      if(getDHTHumidity()) {
        Serial.print("humidityDHT = ");
        Serial.print(humidityDHT);
        Serial.println(" %");
      }
      if (getBMP180()) {
        Serial.print("temperatureBMP = ");
        Serial.print(temperatureBMP);
        Serial.println(" Celsius");

        Serial.print("PressureBMP = ");
        Serial.print(pressureBMP);
        Serial.println(" Pascal");
      }

      if (getWindMeterPerSec()) {
          Serial.print("Wind = ");
          Serial.print(windMeterPerSec);
          Serial.println(" m/s");
          Serial.print("Wind = ");
          Serial.print(getWindKmPerHour());
          Serial.println(" km/h");
          Serial.print("Wind = ");
          Serial.print(getWindDirFirst());
          Serial.println(" ");
          Serial.print("Wind = ");
          Serial.print(getRoseDesVents());
          Serial.println(" ");
      }
      if (getRainMmPerSquareMeter()) {
        Serial.print("Rain = ");
        Serial.print(rainMmPerSquareMeter);
        Serial.println(" mm/m2");
        Serial.print("Rain = ");
        Serial.print(getRainIntensityMmPerMinute());
        Serial.println(" mm/minut");
      }

      if (cmd=='d' || cmd =='j' || cmd =='i') {
        Serial.printf("%s Opt Heap:%u ... \n\r",getDate().c_str(), ESP.getFreeHeap());
      }
      if ( (retJeedom > 0) && (retJeedom != HTTP_CODE_OK) ) {
        saveConfigJeedom = true;
      }
      // Save meteo html.
      String msg = meteoHtml();
      if (cmd=='d')
        Serial.printf("%s ret=%s \n\r", getDate().c_str(), msg.c_str());
    } // end 5 m

    // Optional action
    if (saveConfigJeedom ) {
      boolean scjd = jeedom.saveConfigurationJeedom();
      saveConfigJeedom = false;
      if ( (cmd=='d') && scjd) Serial.printf("%s Configuration Jeedom file has been saved. \n\r", getDate().c_str());
    }


  } // End second
}
