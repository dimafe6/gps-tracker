#ifndef ESP8266HTTPClient_H_
#include <ESP8266HTTPClient.h>
#endif

struct Response {
  int code = 0;
  String responseText;
};

class TrackerApi
{
 public:
    Response post(String endpoint, String data);
    Response get(String endpoint);
 private:
    String apiUrl = "http://gps-tracker.local/api";
    HTTPClient http;
};