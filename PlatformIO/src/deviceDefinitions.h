#ifndef ESP32RHASSPYSATELLITE_DEVICEDEFINITIONS_H
#define ESP32RHASSPYSATELLITE_DEVICEDEFINITIONS_H

int hotword_colors[4] = {0, 255, 0, 0};
int idle_colors[4] = {0, 0, 255, 0};
int wifi_conn_colors[4] = {0, 0, 255, 0};
int wifi_disc_colors[4] = {255, 0, 0, 0};
int ota_colors[4] = {0, 0, 0, 255};
enum {
  COLORS_HOTWORD = 0,
  COLORS_WIFI_CONNECTED = 1,
  COLORS_WIFI_DISCONNECTED = 2,
  COLORS_IDLE = 3,
  COLORS_OTA = 4
};
enum {
  MODE_MIC = 0,
  MODE_SPK = 1
};

//Devices can have several outputs. 
//The Matrix Voice has an jack and a speaker
enum {
  AMP_OUT_SPEAKERS = 0,
  AMP_OUT_HEADPHONE = 1
};

#endif