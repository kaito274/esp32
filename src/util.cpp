#include "util.h"
#include "GlobalSettings.h"
#include <WiFi.h>
#include <ArduinoJson.h>

// Server settings
#ifndef __CONFIG_H__
#define __CONFIG_H__

inline const char *ssid = "YOUR_SSID"; // Replace with your network credentials
inline const char *password = "YOUR_PASSWORD"; // Replace with your network credentials

#endif // __CONFIG_H__

void sendDataTask(void *parameter)
{
  Serial.println("Core 1");
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server_esp32.begin();

  while (1)
  {
    WiFiClient client = server_esp32.available();
    if (client)
    {
      while (client.connected())
      {
        // Serial.println(message);
        client.println(message_socket_car_position);
        for (int i = 0; i < WHEEL_COUNT; i++)
        {
          // Serial.println(message_socket_velocity[i]);
          client.println(message_socket_velocity[i]);
          delay(100);
        }
        delay(100);
      }
      // delay(150);
    }
  }
}