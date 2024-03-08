#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <DHT.h>
#include <DHT_U.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include "data.h"
#include "Settings.h"
#include "UbidotsEsp32Mqtt.h"
#include <UbiConstants.h>
#include <UbiTypes.h>

#define DHTPIN 27 // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE DHT11 // DHT 11

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s
DHT dht(DHTPIN, DHTTYPE);
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
// Publicar Ubidots ##################
const char *UBIDOTS_TOKEN = "BBUS-4A0XexFStypzV1WtTm4rfQydg0Y330"; // Put here your Ubidots TOKEN
// const char *WIFI_SSID = "";      // Put here your Wi-Fi SSID
// const char *WIFI_PASS = "";      // Put here your Wi-Fi password
const char *PUBLISH_DEVICE_LABEL = "esp32"; // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "t";           // Put here your Variable label to which data  will be published
const char *HUMEDAD_VARIABLE_LABEL = "h";
const char *SUBSCRIBE_DEVICE_LABEL = "esp32";  // Replace with the device label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL = "sw1";  // Replace with the variable label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL2 = "sw2"; // Replace with the variable label to subscribe to

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

unsigned long timer;
uint8_t analogPin = 27; // Pin used to read data from GPIO34 ADC_CH6.

Ubidots ubidots(UBIDOTS_TOKEN);

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();
void callback(char *topic, byte *payload, unsigned int length);

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("TTGO_AP", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  // Rutinas de Ubidots en Setup ##########################
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL2);

  timer = millis();
  Serial.println(F("DHTxx test!"));

  dht.begin();
  // Titulos
  tft.init();
  tft.fillScreen(TFT_NAVY);
  tft.drawString("Sensor DHT-11", 10, 15, 2);
  tft.drawString("Humedad: ", 10, 33, 2);
  tft.drawString("Temperatura: ", 10, 90, 2);
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    // loop para Ubidots ##########################

    if (!ubidots.connected())
    {
      ubidots.reconnect();
      ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively
      ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL2);
    }
    if ((millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      float value = analogRead(analogPin);
      Serial.print(value);
      Serial.println();
    }
    ubidots.loop();
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    Serial.print(F("Humedad: "));
    Serial.print(h);
    Serial.println();
    Serial.print(F("% Temperatura: "));
    Serial.print(t);
    Serial.print(F(" C "));
    Serial.println();

    tft.drawString(String(h), 30, 60, 4);
    tft.drawString(String(t), 30, 120, 4);
    ubidots.add(VARIABLE_LABEL, t); // Insert your variable Labels and the value to be sent
    ubidots.add(HUMEDAD_VARIABLE_LABEL, h);
    ubidots.publish(PUBLISH_DEVICE_LABEL);
    timer = millis();
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}
// Callback de trabajos anteriores #######################
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    if ((char)payload[0] == '1')
    {
      Serial.println("on");
      tft.fillCircle(35, 183, 25, TFT_GREEN);
    }
    else
    {
      Serial.println("off");
      tft.fillCircle(35, 183, 25, TFT_RED);
    }
    // Prueba strings
    if (strcmp(topic, "/v2.0/devices/esp32/sw2/lv") == 0)
    {
      tft.fillCircle(90, 183, 25, TFT_RED);
    }
    else
    {
      tft.fillCircle(90, 183, 25, TFT_GREEN);
    }
    Serial.println();
  }
}