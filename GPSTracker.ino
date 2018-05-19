//#define BLYNK_PRINT Serial
#define BLYNK_GREEN     "#23C48E"
#define BLYNK_BLUE      "#04C0F8"
#define BLYNK_YELLOW    "#ED9D00"
#define BLYNK_RED       "#D3435C"

#define FAKE_GPS;
#define NMEA "$GPRMC,192631,A,5025.072260,N,03026.522160,E,5.4,24.7,170518,,,A*42\r\n$GPGGA,192631,5025.072260,N,03026.522160,E,1,12,0.78,3.0,M,0.0,M,,*45\r\n$GPGLL,5025.072260,N,03026.522160,E,192631,A,A*4F\r\n$GPZDA,192631,17,05,2018,,*4E\r\n$GPGSA,A,3,07,25,16,01,18,22,26,30,08,47,24,06,1.32,0.78,1.07*08\r\n$GPGSV,3,1,12,07,90,0,44,25,30,240,40,16,30,120,40,01,30,0,40*75\r\n$GPGSV,3,2,12,18,59,203,42,22,59,278,40,26,15,108,40,30,49,183,41*70\r\n$GPGSV,3,3,12,08,35,057,49,47,32,125,46,24,20,152,47,06,16,274,42*7A\r\n$GPRMC,192641,A,5025.085880,N,03026.531982,E,5.4,24.7,170518,,,A*4F\r\n$GPGGA,192641,5025.085880,N,03026.531982,E,1,12,0.78,3.0,M,0.0,M,,*48\r\n$GPGLL,5025.085880,N,03026.531982,E,192641,A,A*42\r\n$GPZDA,192641,17,05,2018,,*49\r\n$GPRMC,192651,A,5025.099499,N,03026.541803,E,5.4,24.7,170518,,,A*48\r\n$GPGGA,192651,5025.099499,N,03026.541803,E,1,12,0.78,3.0,M,0.0,M,,*4F\r\n$GPGLL,5025.099499,N,03026.541803,E,192651,A,A*45\r\n$GPZDA,192651,17,05,2018,,*48\r\n$GPRMC,192701,A,5025.113119,N,03026.551625,E,5.4,24.7,170518,,,A*49\r\n$GPGGA,192701,5025.113119,N,03026.551625,E,1,12,0.78,3.0,M,0.0,M,,*4E\r\n$GPGLL,5025.113119,N,03026.551625,E,192701,A,A*44\r\n$GPZDA,192701,17,05,2018,,*4C\r\n$GPGSA,A,3,07,25,16,01,18,22,26,30,08,47,24,06,1.32,0.78,1.07*08\r\n$GPGSV,3,1,12,07,90,0,44,25,30,240,40,16,30,120,40,01,30,0,40*75\r\n$GPGSV,3,2,12,18,59,203,42,22,59,278,40,26,15,108,40,30,49,183,41*70\r\n$GPGSV,3,3,12,08,35,057,49,47,32,125,46,24,20,152,47,06,16,274,42*7A\r\n$GPRMC,192711,A,5025.126739,N,03026.561447,E,5.4,24.7,170518,,,A*4F\r\n$GPGGA,192711,5025.126739,N,03026.561447,E,1,12,0.78,3.0,M,0.0,M,,*48\r\n$GPGLL,5025.126739,N,03026.561447,E,192711,A,A*42\r\n$GPZDA,192711,17,05,2018,,*4D\r\n$GPRMC,192721,A,5025.140358,N,03026.571269,E,5.4,24.7,170518,,,A*44\r\n$GPGGA,192721,5025.140358,N,03026.571269,E,1,12,0.78,3.0,M,0.0,M,,*43\r\n$GPGLL,5025.140358,N,03026.571269,E,192721,A,A*49\r\n$GPZDA,192721,17,05,2018,,*4E\r\n$GPRMC,192731,A,5025.153978,N,03026.581091,E,5.4,24.7,170518,,,A*45\r\n$GPGGA,192731,5025.153978,N,03026.581091,E,1,12,0.78,3.0,M,0.0,M,,*42\r\n$GPGLL,5025.153978,N,03026.581091,E,192731,A,A*48\r\n$GPZDA,192731,17,05,2018,,*4F\r\n$GPGSA,A,3,07,25,16,01,18,22,26,30,08,47,24,06,1.32,0.78,1.07*08\r\n$GPGSV,3,1,12,07,90,0,44,25,30,240,40,16,30,120,40,01,30,0,40*75\r\n$GPGSV,3,2,12,18,59,203,42,22,59,278,40,26,15,108,40,30,49,183,41*70\r\n$GPGSV,3,3,12,08,35,057,49,47,32,125,46,24,20,152,47,06,16,274,42*7A\r\n$GPRMC,192741,A,5025.167597,N,03026.590914,E,5.4,24.7,170518,,,A*4C\r\n$GPGGA,192741,5025.167597,N,03026.590914,E,1,12,0.78,3.0,M,0.0,M,,*4B\r\n$GPGLL,5025.167597,N,03026.590914,E,192741,A,A*41\r\n$GPZDA,192741,17,05,2018,,*48\r\n$GPRMC,192751,A,5025.181217,N,03026.600736,E,5.4,24.7,170518,,,A*4E\r\n$GPGGA,192751,5025.181217,N,03026.600736,E,1,12,0.78,3.0,M,0.0,M,,*49\r\n$GPGLL,5025.181217,N,03026.600736,E,192751,A,A*43\r\n$GPZDA,192751,17,05,2018,,*49\r\n$GPRMC,192801,A,5025.194836,N,03026.610558,E,5.4,24.7,170518,,,A*42\r\n$GPGGA,192801,5025.194836,N,03026.610558,E,1,12,0.78,3.0,M,0.0,M,,*45\r\n$GPGLL,5025.194836,N,03026.610558,E,192801,A,A*4F\r\n$GPZDA,192801,17,05,2018,,*43\r\n$GPGSA,A,3,07,25,16,01,18,22,26,30,08,47,24,06,1.32,0.78,1.07*08\r\n$GPGSV,3,1,12,07,90,0,44,25,30,240,40,16,30,120,40,01,30,0,40*75\r\n$GPGSV,3,2,12,18,59,203,42,22,59,278,40,26,15,108,40,30,49,183,41*70\r\n$GPGSV,3,3,12,08,35,057,49,47,32,125,46,24,20,152,47,06,16,274,42*7A\r\n$GPRMC,192811,A,5025.208456,N,03026.620381,E,5.4,24.7,170518,,,A*4E\r\n$GPGGA,192811,5025.208456,N,03026.620381,E,1,12,0.78,3.0,M,0.0,M,,*49\r\n$GPGLL,5025.208456,N,03026.620381,E,192811,A,A*43\r\n$GPZDA,192811,17,05,2018,,*42\r\n$GPRMC,192821,A,5025.222075,N,03026.630203,E,5.4,24.7,170518,,,A*4A\r\n$GPGGA,192821,5025.222075,N,03026.630203,E,1,12,0.78,3.0,M,0.0,M,,*4D\r\n$GPGLL,5025.222075,N,03026.630203,E,192821,A,A*47\r\n$GPZDA,192821,17,05,2018,,*41\r\n$GPRMC,192831,A,5025.235695,N,03026.640026,E,5.4,24.7,170518,,,A*47\r\n$GPGGA,192831,5025.235695,N,03026.640026,E,1,12,0.78,3.0,M,0.0,M,,*40\r\n$GPGLL,5025.235695,N,03026.640026,E,192831,A,A*4A\r\n$GPZDA,192831,17,05,2018,,*40\r\n$GPGSA,A,3,07,25,16,01,18,22,26,30,08,47,24,06,1.32,0.78,1.07*08\r\n$GPGSV,3,1,12,07,90,0,44,25,30,240,40,16,30,120,40,01,30,0,40*75\r\n$GPGSV,3,2,12,18,59,203,42,22,59,278,40,26,15,108,40,30,49,183,41*70\r\n$GPGSV,3,3,12,08,35,057,49,47,32,125,46,24,20,152,47,06,16,274,42*7A\r\n$GPRMC,192841,A,5025.249314,N,03026.649849,E,5.4,24.7,170518,,,A*4F\r\n$GPGGA,192841,5025.249314,N,03026.649849,E,1,12,0.78,3.0,M,0.0,M,,*48\r\n$GPGLL,5025.249314,N,03026.649849,E,192841,A,A*42\r\n$GPZDA,192841,17,05,2018,,*47\r\n$GPRMC,192851,A,5025.262934,N,03026.659672,E,5.4,24.7,170518,,,A*48\r\n$GPGGA,192851,5025.262934,N,03026.659672,E,1,12,0.78,3.0,M,0.0,M,,*4F\r\n$GPGLL,5025.262934,N,03026.659672,E,192851,A,A*45\r\n$GPZDA,192851,17,05,2018,,*46\r\n$GPRMC,192901,A,5025.276553,N,03026.669495,E,5.4,24.7,170518,,,A*4C\r\n$GPGGA,192901,5025.276553,N,03026.669495,E,1,12,0.78,3.0,M,0.0,M,,*4B\r\n$GPGLL,5025.276553,N,03026.669495,E,192901,A,A*41\r\n$GPZDA,192901,17,05,2018,,*42\r\n$GPGSA,A,3,07,25,16,01,18,22,26,30,08,47,24,06,1.32,0.78,1.07*08\r\n$GPGSV,3,1,12,07,90,0,44,25,30,240,40,16,30,120,40,01,30,0,40*75\r\n$GPGSV,3,2,12,18,59,203,42,22,59,278,40,26,15,108,40,30,49,183,41*70\r\n$GPGSV,3,3,12,08,35,057,49,47,32,125,46,24,20,152,47,06,16,274,42*7A\r\n$GPRMC,192911,A,5025.290173,N,03026.679318,E,5.4,24.7,170518,,,A*40\r\n$GPGGA,192911,5025.290173,N,03026.679318,E,1,12,0.78,3.0,M,0.0,M,,*47\r\n$GPGLL,5025.290173,N,03026.679318,E,192911,A,A*4D\r\n$GPZDA,192911,17,05,2018,,*43\r\n"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

extern "C" {
	#include "user_interface.h"
}

#ifdef FAKE_GPS
String nmeaBuff = NMEA;
int nmeaIndex = 0;
#endif

static const int RXPin = D1, TXPin = SW_SERIAL_UNUSED_PIN;
static const uint32_t GPSBaud = 9600;
const int chipSelect = D2;
const int powerSwitchPin = 10;

char auth[] = "a1436b98817c426ea91740829e164a3f";
char ssid[] = "VirtualRouter.codeplex.com";
//char ssid[] = "Zenfone4";
char pass[] = "12345678";

float prevLatitude = 0, prevLongitude = 0;
unsigned long startGPSFindTime = millis();
int blynkConnectionTimer;
int WiFiConnectionTimer;
int blynkGpsTimer;

TinyGPSPlus gps;
BlynkTimer timer;
WidgetLED statusLed(V4);
WidgetMap myMap(V0);
SoftwareSerial ss(RXPin, TXPin);
File root;

//Options
unsigned int GPS_SEARCH_TIME = 5000; //Milliseconds
unsigned long SLEEP_TIME = 30e6; //Microseconds
unsigned int WIFI_CONNECTION_TIMEOUT = 100; //Milliseconds
unsigned int WIFI_CONNECTION_RETRIES = 5;
unsigned int BLYNK_CONNECTION_TIMEOUT = 100; //Milliseconds
unsigned int BLYNK_CONNECTION_RETRIES = 5;
String trackName = "track";
int trackIndex = 0;
bool trackPaused = false;
int frequencyPoints = 100;
bool sleepTypeAuto = true;

void setup() {
	WiFi.persistent( false );

  pinMode(RXPin, INPUT);
  pinMode(powerSwitchPin, OUTPUT);
  digitalWrite(powerSwitchPin, HIGH);

  EEPROM.begin(512);

  Serial.begin(115200);

  readSettingsFromEEPROM();

  ss.begin(GPSBaud);
  ss.enableIntTx(false);

  initSDCard();
  initWiFi();

  checkTraveledDistance();
}

void loop() {
  timer.run();
  if(Blynk.connected()){
    Blynk.run();
  }
}

void initSDCard() {
  Serial.println("");
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    sleep();
  }

  Serial.println("card initialized.");

  if (!SD.exists(getTrackFileName()))
  {
  	Serial.println("Creating new track file " + getTrackFileName());
    File dataFile = SD.open(getTrackFileName(), FILE_WRITE);
    dataFile.println(F("type, satellites, hdop, latitude, longitude, age, date, alt, course, speed, name, desc"));
    dataFile.close();
  }
}

void initWiFi() {
  Serial.println("");

  Serial.println("Connecting to WIFI");
  WiFi.begin(ssid, pass);

	WiFiConnectionTimer = timer.setInterval(WIFI_CONNECTION_TIMEOUT, checkWiFiConnect);
}

void offlineMode() {
  Serial.println("Offline mode");
  wifi_set_sleep_type(MODEM_SLEEP_T);
  delay(1);
}

void checkWiFiConnect() {
  if (WiFi.status() != WL_CONNECTED && WIFI_CONNECTION_RETRIES <= 0) {
  	timer.deleteTimer(WiFiConnectionTimer);
    offlineMode();
    runGPSTimer();
    Serial.println("");

    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
  	timer.deleteTimer(WiFiConnectionTimer);

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Connecting to Blynk server");

    Blynk.config(auth);
    Blynk.connect();

    blynkConnectionTimer = timer.setInterval(BLYNK_CONNECTION_TIMEOUT, checkBlynkConnect);
  }

  WIFI_CONNECTION_RETRIES--;
}

void checkBlynkConnect() {
  if (!Blynk.connected() && BLYNK_CONNECTION_RETRIES <= 0) {
    Serial.println("Blynk connection timeout.");
    timer.deleteTimer(blynkConnectionTimer);
    offlineMode();
    runGPSTimer();

    return;
  }

  if (Blynk.connected()) {
    timer.deleteTimer(blynkConnectionTimer);

    Serial.println("Blynk server connected");
    runGPSTimer();
    Blynk.virtualWrite(V7, "Tracking");
    statusLed.setColor(BLYNK_GREEN);
		statusLed.on();

		return;
  }

  BLYNK_CONNECTION_RETRIES--;
}

void runGPSTimer() {
  Serial.println("Waiting for GPS data...");
  timer.deleteTimer(blynkGpsTimer);
  startGPSFindTime = millis();
  blynkGpsTimer = timer.setInterval(0.01, processGPSData);
}

void sleep() {
	if(SLEEP_TIME > 0) {
		Serial.println("");
		Serial.println("Sleeping...");

		ESP.deepSleep(SLEEP_TIME, WAKE_RF_DEFAULT);
	}
}

void processGPSData() {
  String content = "";
  char character;
  unsigned long currentSearchTime =  millis() - startGPSFindTime;

  #ifdef FAKE_GPS
  	int i = 0;

  	if(nmeaIndex+2 >= nmeaBuff.length()) {
  		nmeaIndex = 0;
  		romWriteInt(44, nmeaIndex);
  	}

		for(i = nmeaIndex; i < nmeaBuff.length(); i++) {
			character = nmeaBuff[i];
	    content.concat(character);
	    if (gps.encode(character)) {
	    	nmeaIndex = i;
	    	romWriteInt(44, nmeaIndex);
	      displayInfo();
	    }
	  }
  #else
	  while (ss.available() && currentSearchTime <= GPS_SEARCH_TIME && trackPaused == false) {
	    character = ss.read();
	    content.concat(character);
	    if (gps.encode(character)) {
	      displayInfo();
	    }
	  }
  #endif

  if (content != "" && Blynk.connected()) {
    Blynk.virtualWrite (V10, content);
  }

  if ( currentSearchTime > GPS_SEARCH_TIME) {
    Serial.println("GPS fix not found.");
    sleep();
  }

  if (trackPaused) {
    Serial.println("Track paused.");
    sleep();
  }
}

void displayInfo() {
  if (fixFound())
  {
    float latitude = (gps.location.lat());     //Storing the Lat. and Lon.
    float longitude = (gps.location.lng());
    float speedMps = gps.speed.mps();
    float alt = gps.altitude.meters();
    int sats = gps.satellites.value();

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

    myMap.location(1, latitude, longitude, "GPS tracker");

    if(checkTraveledDistance()) {
    	writeToSD();
    	sleep();
  	}
  }
}

String getTrackFileName() {
	return trackName + trackIndex + ".gps";
}

bool fixFound() {
	return gps.location.isValid() && gps.speed.isValid() && gps.date.isValid();
}

float calculateMaxIddleTime() {
	return ((GPS_SEARCH_TIME + (WIFI_CONNECTION_TIMEOUT * WIFI_CONNECTION_RETRIES) + (BLYNK_CONNECTION_TIMEOUT * BLYNK_CONNECTION_RETRIES)) / 1000) + (SLEEP_TIME / 1000000); //Seconds. Default 45
}

bool checkTraveledDistance() {
	if(!fixFound()) {
		return false;
	}

	readPreviousCoordinates();

	if(prevLatitude == 0.000000 && prevLongitude == 0.000000) {
		prevLatitude = gps.location.lat();
		prevLongitude = gps.location.lng();
		writePreviousCoordinates(prevLatitude, prevLongitude);
	}

  float speed = ceil(gps.speed.mps());
  unsigned long distance = TinyGPSPlus::distanceBetween(prevLatitude, prevLongitude, gps.location.lat(), gps.location.lng()); //Meters

  if(speed > 1 && sleepTypeAuto) {
		int newSleepTime = ceil(frequencyPoints / speed);
		SLEEP_TIME = newSleepTime <= 2 ? 2e6 : newSleepTime * 1000000;
		romWriteInt(4, SLEEP_TIME);
		Blynk.virtualWrite(V21, newSleepTime);
		Serial.println("New sleep time is: " + String(newSleepTime));
  }

  Serial.println("Traveled distance from previous point: " + String(distance));

  //3000 - higher limit when GPS transferred wrong coordinates
  return (distance >= frequencyPoints)  && (distance <= 3000);
}

void writeToSD() {
	char sz[32];
	sprintf(sz, "%02d-%02d-%02d %02d:%02d:%02d ", gps.date.month(), gps.date.day(), gps.date.year(),  gps.time.hour(),  gps.time.minute(),  gps.time.second());
	File dataFile = SD.open(getTrackFileName(), FILE_WRITE);

	Serial.println();

	dataFile.print("T");dataFile.print(",");
	dataFile.print((!gps.satellites.isValid()) ? 0 : gps.satellites.value()); dataFile.print(",");
	dataFile.print(!gps.hdop.isValid() ? 0 : gps.hdop.value()); dataFile.print(",");
	dataFile.print(!gps.location.isValid() ? 0 : gps.location.lat(), 6); dataFile.print(",");
	dataFile.print(!gps.location.isValid() ? 0 : gps.location.lng(), 6); dataFile.print(",");
	dataFile.print(!gps.location.isValid() ? 0 : gps.location.age()); dataFile.print(",");

	if (!gps.date.isValid() && !gps.time.isValid()) {
	  dataFile.print("0");
	} else {
	  dataFile.print(sz);
	}
	dataFile.print(",");

	dataFile.print(!gps.altitude.isValid() ? 0 : gps.altitude.meters()); dataFile.print(",");
	dataFile.print(!gps.course.isValid() ? 0 : gps.course.deg()); dataFile.print(",");
	dataFile.print(!gps.speed.kmph() ? 0 : gps.speed.kmph(), 2);
	dataFile.print(","); //name
	dataFile.print(","); //desc

	dataFile.println();
	dataFile.flush();
	dataFile.close();

	writePreviousCoordinates(gps.location.lat(), gps.location.lng());
}

void readPreviousCoordinates() {
	EEPROM.get(36, prevLatitude);
  EEPROM.get(40, prevLongitude);
}

void writePreviousCoordinates(float latitude, float longitude) {
	romWriteFloat(36, latitude);
	romWriteFloat(40, longitude);
}

void readSettingsFromEEPROM() {
  EEPROM.get(0, GPS_SEARCH_TIME);
  if (GPS_SEARCH_TIME < 5) {
    GPS_SEARCH_TIME = 5000;
  } else {
    GPS_SEARCH_TIME *= 1000;
  }

  int sleepSeconds;
  EEPROM.get(4, sleepSeconds);
  SLEEP_TIME = sleepSeconds * 1000 * 1000;

  EEPROM.get(8, trackIndex);

  // EEPROM.get(12, WIFI_CONNECTION_TIMEOUT);
  // if (WIFI_CONNECTION_TIMEOUT < 1) {
  //   WIFI_CONNECTION_TIMEOUT = 1000;
  // } else {
  //   WIFI_CONNECTION_TIMEOUT *= 1000;
  // }

  EEPROM.get(16, WIFI_CONNECTION_RETRIES);
  if (WIFI_CONNECTION_RETRIES < 2) {
    WIFI_CONNECTION_RETRIES = 2;
  }

 	// EEPROM.get(20, BLYNK_CONNECTION_TIMEOUT);
  // if (BLYNK_CONNECTION_TIMEOUT < 1) {
  //   BLYNK_CONNECTION_TIMEOUT = 1000;
  // } else {
  //   BLYNK_CONNECTION_TIMEOUT *= 1000;
  // }

  EEPROM.get(24, BLYNK_CONNECTION_RETRIES);
  if (BLYNK_CONNECTION_RETRIES < 2) {
    BLYNK_CONNECTION_RETRIES = 2;
  }

  int sleepType;
  EEPROM.get(28, sleepType);
  sleepTypeAuto = sleepType == 1;


  EEPROM.get(32, frequencyPoints);
  if (frequencyPoints < 10) {
    frequencyPoints = 10;
  }

  readPreviousCoordinates();

	#ifdef FAKE_GPS
	  EEPROM.get(44, nmeaIndex);
	  //+2 for \r\n
	  if (nmeaIndex+2 >= nmeaBuff.length()) {
	    nmeaIndex = 0;
	  }
  #endif

  Serial.print("Prev latitude: ");
  Serial.println(String(prevLatitude, 6));

  Serial.print("Prev longitute: ");
  Serial.println(String(prevLongitude, 6));

  Serial.println("");
  Serial.println("Current settings:");
  Serial.print("GPS find fix time(seconds): ");
  Serial.println(GPS_SEARCH_TIME / 1000); //Seconds

  Serial.print("Sleep time(seconds): ");
  Serial.println(SLEEP_TIME / 1000 / 1000); //Seconds

  Serial.print("Current track file name: ");
	Serial.println(getTrackFileName());

  // Serial.print("WiFi connection timeout(seconds): ");
  // Serial.println(WIFI_CONNECTION_TIMEOUT / 1000); //Seconds

  Serial.print("WiFi connection retries: ");
  Serial.println(WIFI_CONNECTION_RETRIES);

  // Serial.print("Blynk connection timeout(seconds): ");
  // Serial.println(BLYNK_CONNECTION_TIMEOUT / 1000); //Seconds

  Serial.print("Blynk connection retries: ");
  Serial.println(BLYNK_CONNECTION_RETRIES);

	Serial.print("Sleep type: ");
	Serial.println(sleepTypeAuto ? "Auto" : "Time");

  if(sleepTypeAuto) {
			Serial.print("Frequency points: ");
		  Serial.println(frequencyPoints);
  }
}

void romWriteInt(int addr, int var) {
  EEPROM.put(addr, var);
  EEPROM.commit();
}

void romWriteFloat(int addr, float var) {
  EEPROM.put(addr, var);
  EEPROM.commit();
}

BLYNK_CONNECTED() {
  Blynk.syncAll();
  Blynk.virtualWrite(V6, getTrackFileName());
}

BLYNK_WRITE(V20)
{
  GPS_SEARCH_TIME = param.asInt() * 1000;

  romWriteInt(0, param.asInt());
}

BLYNK_WRITE(V21)
{
  SLEEP_TIME = param.asInt() * 1000 * 1000; //Microseconds

  romWriteInt(4, param.asInt());
}

//Commands
BLYNK_WRITE(V5) {
  switch (param.asInt())
  {
    case 1: //Starting new track
      Serial.println("Starting new track...");
      do {
      	trackIndex++;
      } while(SD.exists(getTrackFileName()));

      romWriteInt(8, trackIndex);

      #ifdef FAKE_GPS
	      nmeaIndex = 0;
				romWriteInt(44, nmeaIndex);
			#endif

      //Reset previous coordinates
			writePreviousCoordinates(0.000000, 0.000000);

			Blynk.virtualWrite(V5, 0); //Reset menu
			Blynk.virtualWrite(V6, getTrackFileName());
			Blynk.notify("Started new track");
    	break;
    case 2: //Pause track			
			Blynk.virtualWrite(V5, 0); //Reset menu
			Blynk.virtualWrite(V7, "Paused");
			statusLed.setColor(BLYNK_YELLOW);
			statusLed.on();
			Blynk.notify("Current track paused");
			delay(100);
			trackPaused = true;
    	break;
    case 3: //Resume track
    	trackPaused = false;
    	Blynk.virtualWrite(V5, 0); //Reset menu
    	Blynk.virtualWrite(V7, "Tracking");
			statusLed.setColor(BLYNK_GREEN);
			statusLed.on();
    	Blynk.notify("Current track resumed");
    	break;
  }
}

// BLYNK_WRITE(V22)
// {
//   WIFI_CONNECTION_TIMEOUT = param.asInt() * 1000; //Milliseconds

//   romWriteInt(12, param.asInt());

//   Serial.print("WiFi connection timeout(seconds): ");
//   Serial.println(param.asInt());
// }

BLYNK_WRITE(V23)
{
  WIFI_CONNECTION_RETRIES = param.asInt();

  romWriteInt(16, param.asInt());
}

// BLYNK_WRITE(V24)
// {
//   BLYNK_CONNECTION_TIMEOUT = param.asInt() * 1000; //Milliseconds

//   romWriteInt(20, param.asInt());

//   Serial.print("Blynk connection timeout(seconds): ");
//   Serial.println(param.asInt());
// }

BLYNK_WRITE(V25)
{
  BLYNK_CONNECTION_RETRIES = param.asInt();

  romWriteInt(24, param.asInt());
}

BLYNK_WRITE(V26)
{
  sleepTypeAuto = param.asInt() == 1;

  romWriteInt(28, sleepTypeAuto);
}

BLYNK_WRITE(V27)
{
  frequencyPoints = param.asInt();

  romWriteInt(32, frequencyPoints);
}

