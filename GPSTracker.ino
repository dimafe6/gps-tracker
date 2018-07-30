//#define FAKE_GPS;
#ifdef FAKE_GPS;
String NMEA[10] {
  "$GPRMC,192631,A,5025.072260,N,03026.522160,E,5.4,24.7,170518,,,A*42\r\n$GPGGA,192631,5025.072260,N,03026.522160,E,1,12,0.78,3.0,M,0.0,M,,*45\r\n$GPGLL,5025.072260,N,03026.522160,E,192631,A,A*4F\r\n$GPZDA,192631,17,05,2018,,*4E\r\n",
  "$GPRMC,192641,A,5025.085880,N,03026.531982,E,5.4,24.7,170518,,,A*4F\r\n$GPGGA,192641,5025.085880,N,03026.531982,E,1,12,0.78,3.0,M,0.0,M,,*48\r\n$GPGLL,5025.085880,N,03026.531982,E,192641,A,A*42\r\n$GPZDA,192641,17,05,2018,,*49\r\n",
  "$GPRMC,192651,A,5025.099499,N,03026.541803,E,5.4,24.7,170518,,,A*48\r\n$GPGGA,192651,5025.099499,N,03026.541803,E,1,12,0.78,3.0,M,0.0,M,,*4F\r\n$GPGLL,5025.099499,N,03026.541803,E,192651,A,A*45\r\n$GPZDA,192651,17,05,2018,,*48\r\n",
  "$GPRMC,192701,A,5025.113119,N,03026.551625,E,5.4,24.7,170518,,,A*49\r\n$GPGGA,192701,5025.113119,N,03026.551625,E,1,12,0.78,3.0,M,0.0,M,,*4E\r\n$GPGLL,5025.113119,N,03026.551625,E,192701,A,A*44\r\n$GPZDA,192701,17,05,2018,,*4C\r\n",
  "$GPRMC,192711,A,5025.126739,N,03026.561447,E,5.4,24.7,170518,,,A*4F\r\n$GPGGA,192711,5025.126739,N,03026.561447,E,1,12,0.78,3.0,M,0.0,M,,*48\r\n$GPGLL,5025.126739,N,03026.561447,E,192711,A,A*42\r\n$GPZDA,192711,17,05,2018,,*4D\r\n",
  "$GPRMC,192721,A,5025.140358,N,03026.571269,E,5.4,24.7,170518,,,A*44\r\n$GPGGA,192721,5025.140358,N,03026.571269,E,1,12,0.78,3.0,M,0.0,M,,*43\r\n$GPGLL,5025.140358,N,03026.571269,E,192721,A,A*49\r\n$GPZDA,192721,17,05,2018,,*4E\r\n",
  "$GPRMC,192731,A,5025.153978,N,03026.581091,E,5.4,24.7,170518,,,A*45\r\n$GPGGA,192731,5025.153978,N,03026.581091,E,1,12,0.78,3.0,M,0.0,M,,*42\r\n$GPGLL,5025.153978,N,03026.581091,E,192731,A,A*48\r\n$GPZDA,192731,17,05,2018,,*4F\r\n",
  "$GPRMC,192741,A,5025.167597,N,03026.590914,E,5.4,24.7,170518,,,A*4C\r\n$GPGGA,192741,5025.167597,N,03026.590914,E,1,12,0.78,3.0,M,0.0,M,,*4B\r\n$GPGLL,5025.167597,N,03026.590914,E,192741,A,A*41\r\n$GPZDA,192741,17,05,2018,,*48\r\n",
  "$GPRMC,192751,A,5025.181217,N,03026.600736,E,5.4,24.7,170518,,,A*4E\r\n$GPGGA,192751,5025.181217,N,03026.600736,E,1,12,0.78,3.0,M,0.0,M,,*49\r\n$GPGLL,5025.181217,N,03026.600736,E,192751,A,A*43\r\n$GPZDA,192751,17,05,2018,,*49\r\n",
  "$GPRMC,192801,A,5025.194836,N,03026.610558,E,5.4,24.7,170518,,,A*42\r\n$GPGGA,192801,5025.194836,N,03026.610558,E,1,12,0.78,3.0,M,0.0,M,,*45\r\n$GPGLL,5025.194836,N,03026.610558,E,192801,A,A*4F\r\n$GPZDA,192801,17,05,2018,,*43\r\n"
};
#endif

#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "FS.h"
#include <ESP8266FtpServer.h>

extern "C" {
	#include "user_interface.h"
}

static const int RXPin = D1, TXPin = SW_SERIAL_UNUSED_PIN;
static const uint32_t GPSBaud = 9600;
const int powerSwitchPin = 10;
const int blynkButtonPin = 12; // D6
const int greenLEDPin = D5;

String ftp_user = "gps-tracker";
String ftp_pass = "gps-tracker";
char auth[] = "a1436b98817c426ea91740829e164a3f";
//char ssid[] = "VirtualRouter.codeplex.com";
char ssid[] = "Zenfone4";
char pass[] = "12345678";
//char ssid[] = "dimaPC";
//char pass[] = "I0U5cRX3";

unsigned long startGPSFindTime = millis();
int WiFiConnectionTimer;
int blynkConnectionTimer;
int gpsTimer;
unsigned int wifiConnectionRetries;
unsigned int blynkConnectionRetries;
String newTrackName = "";
bool blynkMode = false;

FtpServer ftpSrv; 
SimpleTimer timer;
SimpleTimer timerLed;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
File root;

struct Config {
  float prevLatitude = 0;
  float prevLongitude = 0;
  unsigned int gpsSearchTime = 5000;
  unsigned long sleepTime = 2e6;
  unsigned int wifiConnectionTimeout = 100;
  unsigned int wifiConnectionRetries = 50;
  unsigned int blynkConnectionTimeout = 100;
  unsigned int blynkConnectionRetries = 50;
  String currentTrackName = "track0.gps";
  unsigned int frequencyWaypoints = 100;
};

const char *configFilename = "/CONFIG.TXT";
Config config;

void setup() {
  pinMode(greenLEDPin, OUTPUT);
  pinMode(blynkButtonPin, INPUT_PULLUP);
  pinMode(RXPin, INPUT);
  pinMode(powerSwitchPin, OUTPUT);

  Serial.begin(115200);

  blynkMode = digitalRead(blynkButtonPin) == LOW;

  initSPIFFS();

  if(!blynkMode) {
    timerLed.setInterval(500, blinkConfigLED);

    WiFi.forceSleepBegin();
    WiFi.mode(WIFI_OFF);

    digitalWrite(powerSwitchPin, HIGH);
    
    ss.begin(GPSBaud);
    ss.enableIntTx(false);

    runGPSTimer();
  } else {
    timerLed.setInterval(100, blinkConfigLED);
    WiFi.persistent(false);
    initWiFi();
  }
}

void loop() {
  timer.run();
  timerLed.run();

  if(blynkMode) {
    Blynk.run();
  }

  ftpSrv.handleFTP();
}

void blinkConfigLED() {
  digitalWrite(greenLEDPin, !digitalRead(greenLEDPin));
}

void initSPIFFS() {
  Serial.println("");
  Serial.print("Initializing SPIFFS...");

  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS initialization failed");
    sleep();
  }

  Serial.println("SPIFFS initialized.");

  // Load last saved configuration
  loadConfiguration(configFilename, config);

  if (!SPIFFS.exists(getTrackFileName()))
  {
  	Serial.println("Creating new track file " + getTrackName());
    File dataFile = SPIFFS.open(getTrackFileName(), "w+");
    dataFile.println(F("latitude, longitude, date, alt, speed"));
    dataFile.close();
  }
}

void initWiFi() {
  Serial.println("");

  Serial.println("Connecting to WIFI");
  WiFi.begin(ssid, pass);

  wifiConnectionRetries = config.wifiConnectionRetries;
	WiFiConnectionTimer = timer.setInterval(config.wifiConnectionTimeout, checkWiFiConnect);
}

void initFTPServer() {
  ftpSrv.begin(ftp_user,ftp_pass);
  Serial.println("FTP server started");

  Blynk.virtualWrite(V23, "Started");
  Blynk.virtualWrite(V19, WiFi.localIP().toString());
  Blynk.virtualWrite(V20, ftp_user);
  Blynk.virtualWrite(V21, ftp_pass);
  Blynk.virtualWrite(V22, "21");
}

void checkWiFiConnect() {
  if (WiFi.status() != WL_CONNECTED && wifiConnectionRetries <= 0) {
    onWiFiConnectionTimeout();

    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    onWiFiConnected();

    return;
  }

  wifiConnectionRetries--;
}

void onWiFiConnected() {
  	timer.deleteTimer(WiFiConnectionTimer);

    Serial.println("WiFi connected");
    Serial.print("IP address: ");Serial.println(WiFi.localIP());    

    Blynk.config(auth);
    Blynk.connect();

    blynkConnectionRetries = config.blynkConnectionRetries;
    blynkConnectionTimer = timer.setInterval(config.blynkConnectionTimeout, checkBlynkConnect);
}

void checkBlynkConnect() {
  if (!Blynk.connected() && blynkConnectionRetries <= 0) {
    onBlynkConnectionTimeout();

    return;
  }

  if (Blynk.connected()) {
    onBlynkConnected();

		return;
  }

  blynkConnectionRetries--;
}

void onBlynkConnectionTimeout() {
  timer.deleteTimer(blynkConnectionTimer);

  Serial.println("Blynk connection timeout.");
  
  offlineMode();
  runGPSTimer();
  initFTPServer();
}

void onBlynkConnected() {
  timer.deleteTimer(blynkConnectionTimer);

  Serial.println("Blynk server connected");
  runGPSTimer();
  initFTPServer();
}

void onWiFiConnectionTimeout() {
  timer.deleteTimer(WiFiConnectionTimer);
  Serial.println("");
  offlineMode();

  runGPSTimer();
}

void loadConfiguration(JsonObject &root, Config &config) {
  config.prevLatitude = root["prevLatitude"] | config.prevLatitude;
  config.prevLongitude = root["prevLongitude"] | config.prevLongitude;
  config.gpsSearchTime = root["gpsSearchTime"] | config.gpsSearchTime;
  config.sleepTime = root["sleepTime"] | config.sleepTime;
  config.wifiConnectionTimeout = root["wifiConnectionTimeout"] | config.wifiConnectionTimeout;
  config.wifiConnectionRetries = root["wifiConnectionRetries"] | config.wifiConnectionRetries;
  config.blynkConnectionTimeout = root["blynkConnectionTimeout"] | config.blynkConnectionTimeout;
  config.blynkConnectionRetries = root["blynkConnectionRetries"] | config.blynkConnectionRetries;
  String currentTrackName = root["currentTrackName"].as<String>();
  config.currentTrackName = currentTrackName == "" ? config.currentTrackName : currentTrackName;
  config.frequencyWaypoints = root["frequencyWaypoints"] | config.frequencyWaypoints;

  root.printTo(Serial);
}

void loadConfiguration(const char *filename, Config &config) {
  Serial.println("Loading configuration from SPIFFS...");

  File file = SPIFFS.open(filename, "r");

  // Allocate the memory pool on the stack.
  // Don't forget to change the capacity to match your JSON document.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonBuffer<1024> jsonBuffer;

  JsonObject &root = jsonBuffer.parseObject(file);

  if (!root.success()) {
    Serial.println(F("Failed to read file, using default configuration"));
  }

  loadConfiguration(root, config);
  
  file.close();
}

// Saves the configuration to a file
void saveConfiguration(const char *filename, const Config &config, bool print) {
  if(print) {
    Serial.println("");
    Serial.println("Saving configuration...");  
  }

  File file = SPIFFS.open(filename, "w+");
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  StaticJsonBuffer<1024> jsonBuffer;

  JsonObject &root = jsonBuffer.createObject();

  root["prevLatitude"] = config.prevLatitude;
  root["prevLongitude"] = config.prevLongitude;
  root["gpsSearchTime"] = config.gpsSearchTime;
  root["sleepTime"] = config.sleepTime;
  root["wifiConnectionTimeout"] = config.wifiConnectionTimeout;
  root["wifiConnectionRetries"] = config.wifiConnectionRetries;
  root["blynkConnectionTimeout"] = config.blynkConnectionTimeout;
  root["blynkConnectionRetries"] = config.blynkConnectionRetries;
  root["currentTrackName"] = config.currentTrackName;
  root["frequencyWaypoints"] = config.frequencyWaypoints;

  if (root.printTo(file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  if(print) {
    root.printTo(Serial);
  }

  Serial.println("");
  Serial.println("Saved");
  
  file.close();
}

void offlineMode() {
  Serial.println("Offline mode");
  ESP.deepSleep(1e1, WAKE_RF_DEFAULT);
}

void runGPSTimer() {
  Serial.println();
  Serial.println("Waiting for GPS data...");
  stopGPSTimer();
  startGPSFindTime = millis();
  gpsTimer = timer.setInterval(0.005, processGPSData);
}

bool sleep() {
  Blynk.virtualWrite(V23, "Stopped");

  if(blynkMode) {
    return false;
  }

  saveConfiguration(configFilename, config, true);

	if(config.sleepTime > 0) {
		Serial.println("");
		Serial.println("Sleeping...");

		ESP.deepSleep(config.sleepTime, WAKE_RF_DEFAULT);
	}
}

void stopGPSTimer() {
  timer.deleteTimer(gpsTimer);
}

void processGPSData() {
  String content = "";
  char character;
  unsigned long currentSearchTime =  millis() - startGPSFindTime;

  #ifdef FAKE_GPS
  	int i = 0;

    int index = config.lastMapPointIndex;
    if(index >= 10) {
      config.lastMapPointIndex = 0;
      index = 0;
    }

    String nmeaBuff = NMEA[index];
		for(i = 0; i < nmeaBuff.length() - 1; i++) {
			character = nmeaBuff[i];
	    if (gps.encode(character)) {       
	      displayInfo();
	    }
      delay(2);
	  }
    config.lastMapPointIndex++;
  #else
	  while (ss.available() && currentSearchTime <= config.gpsSearchTime) {
	    character = ss.read();
	    content.concat(character);
	    if (gps.encode(character)) {
	      displayInfo();
	    }
	  }
  #endif

  if ( currentSearchTime > config.gpsSearchTime) {
    Serial.println("GPS fix not found.");
    if(!sleep()) {
      startGPSFindTime = millis();
    }
  }
}

void displayInfo() {
  if (fixFound())
  {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    float speedMps = gps.speed.mps();
    float alt = gps.altitude.meters();
    int sats = gps.satellites.value();

    if(checkTraveledDistance()) {
      Serial.print("LAT:  ");
      Serial.println(latitude, 6);
      Serial.print("LONG: ");
      Serial.println(longitude, 6);
      Serial.print("SPEED: ");
      Serial.println(speedMps, 2);
      Serial.print("ALT: ");
      Serial.println(alt, 2);
      Serial.print("SATS: ");
      Serial.println(sats, 1);

      writeToSPIFF();

      sleep();
    }
  }
}

String getTrackFileName() {
	return "/" + config.currentTrackName;
}

String getTrackName() {
	return config.currentTrackName;
}

bool fixFound() {
	return gps.location.isValid() && gps.speed.isValid() && gps.date.isValid();
}

float calculateMaxIddleTime() {
	return ((config.gpsSearchTime + (config.wifiConnectionTimeout * config.wifiConnectionRetries)) / 1000) + (config.sleepTime / 1000000); //Seconds.
}

bool checkTraveledDistance() {
	if(!fixFound()) {
		return false;
	}

	if(config.prevLatitude == 0 && config.prevLongitude == 0) {
		config.prevLatitude = gps.location.lat();
		config.prevLongitude = gps.location.lng();
	}

  float speed = ceil(gps.speed.mps());
  unsigned long distance = TinyGPSPlus::distanceBetween(config.prevLatitude, config.prevLongitude, gps.location.lat(), gps.location.lng()); //Meters

  if(speed > 1 && config.frequencyWaypoints > 0) {
		int newSleepTime = ceil(config.frequencyWaypoints / speed);
		config.sleepTime = newSleepTime <= 2 ? 2e6 : newSleepTime * 1000000;
		Serial.println("New sleep time is: " + String(newSleepTime));
  }

  Serial.println("Traveled distance from previous point: " + String(distance));

  //3000 - higher limit when GPS transferred wrong coordinates
  return (distance >= config.frequencyWaypoints)  && (distance <= 3000);
}

void writeToSPIFF() {
	char date[32];
	sprintf(date, "%02d-%02d-%02d %02d:%02d:%02d ", gps.date.month(), gps.date.day(), gps.date.year(),  gps.time.hour(),  gps.time.minute(),  gps.time.second());
	File dataFile = SPIFFS.open(getTrackFileName(), "a+");

  config.prevLatitude = gps.location.lat();
  config.prevLongitude = gps.location.lng();

	Serial.println();

	dataFile.print(!gps.location.isValid() ? 0 : gps.location.lat(), 6); dataFile.print(",");
	dataFile.print(!gps.location.isValid() ? 0 : gps.location.lng(), 6); dataFile.print(",");

	if (!gps.date.isValid() && !gps.time.isValid()) {
	  dataFile.print("0");
	} else {
	  dataFile.print(date);
	}
	dataFile.print(",");

	dataFile.print(!gps.altitude.isValid() ? 0 : gps.altitude.meters()); dataFile.print(",");
	dataFile.print(!gps.speed.kmph() ? 0 : gps.speed.kmph(), 2);dataFile.print(",");

	dataFile.println();
	dataFile.flush();
	dataFile.close();

  Serial.println("Location saved to SPIFFS.");
}

BLYNK_CONNECTED() {
  Blynk.syncAll();
  Blynk.virtualWrite(V7, getTrackName());
}

BLYNK_WRITE(V1) {
  config.gpsSearchTime = param.asInt() * 1000;

  saveConfiguration(configFilename, config, false); 
}

BLYNK_WRITE(V2) {
  config.sleepTime = param.asInt() * 1000 * 1000;

  saveConfiguration(configFilename, config, false);
}

BLYNK_WRITE(V3){
  config.wifiConnectionTimeout = param.asInt();

  saveConfiguration(configFilename, config, false);
}

BLYNK_WRITE(V4){
  config.wifiConnectionRetries = param.asInt(); 

  saveConfiguration(configFilename, config, false);
}

BLYNK_WRITE(V5){
  config.blynkConnectionTimeout = param.asInt(); 

  saveConfiguration(configFilename, config, false);
}

BLYNK_WRITE(V6){
  config.blynkConnectionRetries = param.asInt(); 

  saveConfiguration(configFilename, config, false);
}

BLYNK_WRITE(V9){
  config.frequencyWaypoints = param.asInt(); 

  saveConfiguration(configFilename, config, false);
}

BLYNK_WRITE(V12){
  if(param.asInt() == 1 && newTrackName != "") {
    if(SPIFFS.exists("/" + newTrackName)) {
      newTrackName += "_";
    }

    config.currentTrackName = newTrackName + ".gps";
    config.prevLatitude = 0;
    config.prevLongitude = 0;
    
    Serial.println("Creating new track file " + getTrackName());
    File dataFile = SPIFFS.open(getTrackFileName(), "w+");
    dataFile.println(F("type, latitude, longitude, date, alt, speed, name, desc"));
    dataFile.close();

    Blynk.virtualWrite(V7, getTrackName());
    Blynk.notify("Started new track " + getTrackName());

    Blynk.virtualWrite(V12, 0);

    newTrackName = "";

    saveConfiguration(configFilename, config, false);
  }
}

BLYNK_WRITE(V13){
  newTrackName = param.asString();

  saveConfiguration(configFilename, config, false);
}