#include "TrackerApi.h"

Response TrackerApi::post(String endpoint, String data)
{
    Response response;

    if(endpoint[0] != '/') {
        endpoint = '/' + endpoint;
    }

    String url = apiUrl + endpoint;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    response.code = http.POST(data);
    response.responseText = http.getString();
  
    http.end();

    return response;
};

Response TrackerApi::get(String endpoint)
{
    Response response;

    if(endpoint[0] != '/') {
        endpoint = '/' + endpoint;
    }

    String url = apiUrl + endpoint;
    http.begin(url);
    response.code = http.GET();
    response.responseText = http.getString();
  
    http.end();

    return response;
};