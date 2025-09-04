// secrets.h
#pragma once

// ---- WiFi ----
const char* WIFI_SSID = "Dlink12";
const char* WIFI_PASSWORD = "4d9a4d4652";

// ---- MQTT ----
const char* MQTT_HOST = "192.168.4.23";  // or broker hostname
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENTID = "espresso-1";  // must be unique per client
const char* MQTT_USER = "mqtt-user";       // leave "" if not needed
const char* MQTT_PASS = "0pl,mko9";        // leave "" if not needed

// ---- Topics ----
const char* MQTT_TOPIC = "homeassistant/espresso/telemetry";  // telemetry JSON
const char* MQTT_STATUS = "homeassistant/espresso/status";    // online/offline retained
