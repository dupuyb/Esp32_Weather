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
    DynamicJsonDocument rootcfg(500);
    rootcfg["host"] = config.host;
    rootcfg["port"] = config.port;
    rootcfg["apiKey"] = config.apiKey;
    rootcfg["tempoffset"] = config.tempoffset;
    rootcfg["altitude"] = config.altitude;
    rootcfg["diroffset"] = config.diroffset;
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
    virtualbaseurl += "&type=virtual&id=";
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
