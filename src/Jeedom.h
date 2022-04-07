#ifndef Jeedom_h
#define Jeedom_h

#include <HTTPClient.h>

// configuration file jeedom
struct ConfigJeedom {
  char  host[20];
  int   port;
  char  apiKey[40];
  float tempoffset;
  float altitude;
  int   diroffset;
  int idTemp; // 1925; // Température
  int idHumi; // 1926; // % humidity
  int idBaro; // 1927; // Pression barometric
  int idBars; // 1935; // Pression barometric
  int idAlti; // 1936; // Altidute
  int idWmps; // 1937; // Wind m/s
  int idWkph; // 1938; // Wind km/h
  int idRmpm; // 1939; // Rain mm/m2/h 
  int idRjou; // 1940; // Rain intensity per jpour
  int idWdir; // 1941; // Wind direction
};

class Jeedom {
public:

  boolean isCcrChanged(){
     return (myCrc8((uint8_t*)&config, sizeof(config) - 1) != ccrConfig);
  }

  boolean saveConfigurationJeedom() {
    File file = SPIFFS.open(fileconfigjeedom, "w");
    if (!file) {
       return false;
    }
    String cfJeedomjson;
    // ArduinoJson 6
    DynamicJsonDocument rootcfg(1024);
    rootcfg["host"] = config.host;
    rootcfg["port"] = config.port;
    rootcfg["apiKey"] = config.apiKey;
    rootcfg["tempoffset"] = config.tempoffset;
    rootcfg["altitude"] = config.altitude;
    rootcfg["diroffset"] = config.diroffset;
    rootcfg["idTemp"] = config.idTemp;
    rootcfg["idHumi"] = config.idHumi;
    rootcfg["idBaro"] = config.idBaro;
    rootcfg["idBars"] = config.idBars;
    rootcfg["idAlti"] = config.idAlti;
    rootcfg["idWmps"] = config.idWmps;
    rootcfg["idWkph"] = config.idWkph;
    rootcfg["idRmpm"] = config.idRmpm;
    rootcfg["idRjou"] = config.idRjou;
    rootcfg["idWdir"] = config.idWdir;
    // ArduinoJson 6
    serializeJson(rootcfg, cfJeedomjson);
    file.print(cfJeedomjson);
    file.close();
    ccrConfig = getCcrConfig();
    return true;
  }

  void loadConfigurationJeedom() {
    // Open file for reading configuration
    File file = SPIFFS.open(fileconfigjeedom, "r");
    if (!file) {
      Serial.println(F("Configuration Jeedom file is absent."));
    } else {
      size_t size = file.size();
      std::unique_ptr<char[]> buf(new char[size]);
      file.readBytes(buf.get(), size);
      // ArduinoJson 6
      DynamicJsonDocument rootcfg(1024);
      auto error = deserializeJson(rootcfg, buf.get());
      strlcpy(config.host, rootcfg["host"] | "192.168.1.117", sizeof(config.host));
      config.port = rootcfg["port"] | 80;
      strlcpy(config.apiKey, rootcfg["apiKey"] | "unknown", sizeof(config.apiKey));
      config.tempoffset = rootcfg["tempoffset"] | -5.0; 
      config.altitude = rootcfg["altitude"] | 455.0; 
      config.diroffset = rootcfg["diroffset"] | 0;
      config.idTemp = rootcfg["idTemp"] | 1925; // Température
      config.idHumi = rootcfg["idHumi"] | 1926; // % humidity
      config.idBaro = rootcfg["idBaro"] | 1927; // Pression barometric
      config.idBars = rootcfg["idBars"] | 1935; // Pression barometric
      config.idAlti = rootcfg["idAlti"] | 1936; // Altidute
      config.idWmps = rootcfg["idWmps"] | 1937; // Wind m/s
      config.idWkph = rootcfg["idWkph"] | 1938; // Wind km/h
      config.idRmpm = rootcfg["idRmpm"] | 1939; // Rain mm/m2/h 
      config.idRjou = rootcfg["idRjou"] | 1940; // Rain intensity per jpour
      config.idWdir = rootcfg["idWdir"] | 1941; // Wind direction
      if (error)  saveConfigurationJeedom();
    }
    ccrConfig = getCcrConfig();
  }

  Jeedom(const char * filename) {
    fileconfigjeedom = filename;
  };

  void setup() {
    jeeComErr = 0;
    loadConfigurationJeedom();
    virtualbaseurl = "/core/api/jeeApi.php?apikey=";
    virtualbaseurl += config.apiKey;
    virtualbaseurl += "&type=event&plugin=virtual&id=";
  }

  int sendVirtual(int id, float val) {
    char temp[20]; // force for m3 in liter
    snprintf(temp, 20, "%.3f", val);
    String url = virtualbaseurl + String(id);
    url += url + "&value="; url += String(temp);
    http.begin(config.host,config.port, url);
    int httpCode = http.GET();
    
    if (httpCode!=HTTP_CODE_OK) jeeComErr++;
    else jeeComErr = 0;
    http.end();
    return httpCode;
  }

  int getErrorCounter() {
    return jeeComErr;
  }

private :
  uint8_t myCrc8(uint8_t * data, uint8_t count) {
    uint8_t result = 0xDF;
    while (count--) {
      result ^= *data;
      data++;
    }
    return result;
  }

  uint8_t getCcrConfig() {
    return myCrc8((uint8_t*)&config, sizeof(config) - 1);
  }

public:
  uint8_t ccrConfig;
  ConfigJeedom config;

private:
  const char * fileconfigjeedom;
  HTTPClient http;
  String virtualbaseurl;
  int jeeComErr;

};

#endif
