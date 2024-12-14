/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-web-server-dc-motor-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include "Server.h"
#include "Motor.h"
#include "GlobalSettings.h"

// Replace with your network credentials
static const char *ssid = "BS HUY - Deco";
static const char *password = "khongcopass";

// Create AsyncWebServer object on port 80
static AsyncWebServer server(80);

// HTML and CSS to build the web page
static const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html>
    <head>
      <title>ESP IOT DASHBOARD</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <link rel="stylesheet" type="text/css" href="style.css">
      <link rel="icon" type="image/png" href="favicon.png">
      <script src="https://kit.fontawesome.com/0294e3a09e.js" crossorigin="anonymous"></script>
      <style>
        html {
        font-family: Arial, Helvetica, sans-serif;
        text-align: center;
        }
        h1 {
            font-size: 1.8rem;
            color: white;
        }
        .topnav {
            overflow: hidden;
            background-color: #0A1128;
        }
        body {
            margin: 0;
        }
        .content {
            padding: 50px;
        }
        .card-grid {
            max-width: 800px;
            margin: 0 auto;
            display: grid;
            grid-gap: 2rem;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        }
        .card {
            background-color: white;
            box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
        }
        .card-title {
            font-size: 1.2rem;
            font-weight: bold;
            color: #034078
        }
        .state {
            font-size: 1.2rem;
            color:#1282A2;
        }
        button {
            border: none;
            color: #FEFCFB;
            padding: 15px 32px;
            text-align: center;
            font-size: 16px;
            width: 150px;
            border-radius: 4px;
            transition-duration: 0.4s;
        }
        .button-on {
            background-color:#034078;
        }
        .button-on:hover {
            background-color: #1282A2;
        }
        .button-off {
            background-color:#858585;
        }
        .button-off:hover {
            background-color: #252524;
        }
        .button-stop {
            background-color:#5e0f0f;
            width: 100%;
        }
        .button-stop:hover {
            background-color: #9b332c;
        }
      </style>
    </head>
    <body>
      <div class="topnav">
        <h1>CONTROL DC MOTOR</h1>
      </div>
      <div class="content">
        <div class="card-grid">
          <div class="card">
            <p class="card-title"><i class="fa-solid fa-gear"></i> DC Motor A</p>
            <p>
              <a href="change_rpm"><button class="button-on"><i class="fa-solid fa-arrow-up"></i> CHANGE RPM</button></a>
              <button id="moveButton" class="button-off"><i class="fa-solid fa-arrow-down"></i> MOVE</button>
            </p>
            <p>
              // <a href="go_straight" ><button class="button-on"><i class="fa-solid fa-arrow-right"></i> GO STRAIGHT</button></a>
              <button id="goStraight" class="button-off"><i class="fa-solid fa-arrow-right"></i> GO STRAIGHT</button>
            </p>
            <p>
              <a href="stop"><button class="button-stop"><i class="fa-solid fa-stop"></i> STOP</button></a>
            </p>
          </div>
        </div>
      </div>

      <script>
        // Get the move button
        const moveButton = document.getElementById('moveButton');
        const goStraight = document.getElementById('goStraight');

        // When the button is pressed, send a request to start the motor
        moveButton.addEventListener('mousedown', function() {
          fetch('/move', { method: 'GET' })
            .then(response => response.text())
            .then(data => {
              // Optionally, you can update the UI to reflect motor running
              moveButton.classList.add('button-on');  // Change button style (example)
            });
        });

        // When the button is pressed, send a request to start the motor
        goStraight.addEventListener('mousedown', function() {
          fetch('/go_straight', { method: 'GET' })
            .then(response => response.text())
            .then(data => {
              // Optionally, you can update the UI to reflect motor running
              goStraight.classList.add('button-on');  // Change button style (example)
            });
        });

        // When the button is released, stop the motor
        moveButton.addEventListener('mouseup', function() {
          fetch('/stop', { method: 'GET' })
            .then(response => response.text())
            .then(data => {
              // Optionally, you can update the UI to reflect motor stopped
              moveButton.classList.remove('button-on');  // Reset button style
            });
        });

        // In case the user moves the mouse away from the button while holding it
        moveButton.addEventListener('mouseleave', function() {
          fetch('/stop', { method: 'GET' })
            .then(response => response.text())
            .then(data => {
              // Optionally, reset button style
              moveButton.classList.remove('button-on');
            });
        });

        goStraight.addEventListener('mouseup', function() {
          fetch('/stop', { method: 'GET' })
            .then(response => response.text())
            .then(data => {
              // Optionally, reset button style
              goStraight.classList.remove('button-on');
            });
        });
      </script>
    </body>
  </html>
)rawliteral";

int speed = 50;
void initWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

int arrSetpoints[] = {50, 100, 150};
int cur = 0;

// change the speed of the motor
void changeRPM()
{
  cur = (cur + 1) % 3;
  Serial.print("Cur: ");
  Serial.println(cur);
  Serial.print("Setpoint: ");
  Serial.println(arrSetpoints[cur]);
  wheelsSpeed[0].setTargetRPM(arrSetpoints[cur]);
}

void move()
{
  // Logic to start the motor
  // For example, turn on the motor and LED
  digitalWrite(2, HIGH); // Assuming you have a LED connected
  // wheelsSpeed[0].getMotor().Enable();
  dir = -1;
}

void goStraight()
{
  digitalWrite(2, HIGH);              // Assuming you have a LED connected
  // wheelsSpeed[0].getMotor().Enable(); // Enable the second motor.
  dir = 1;
}

void stopMotor()
{
  // Logic to stop the motor
  // For example, turn off the motor and LED
  digitalWrite(2, LOW); // Turn off LED when motor is stopped
  wheelsSpeed[0].getMotor().Disable();
}

void startServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });

  server.on("/change_rpm", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    changeRPM();
    request->send_P(200, "text/html", index_html); });

  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    move();
    request->send_P(200, "text/html", index_html); });

  server.on("/go_straight", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    goStraight();
    request->send_P(200, "text/html", index_html); });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    stopMotor();
    request->send_P(200, "text/html", index_html); });

  server.begin();
}
