#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "esp_wpa2.h"
#include "esp_wifi.h"
#include <PubSubClient.h>
#include "config.h"
// #include <WiFiClientSecure.h>

#define Serial USBSerial
#define PIN_R 18 

void connectWiFi() {
    Serial.print("Attemping WiFi connection");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);

    // the line below is for connecting to an 'open' network
    // WiFi.begin(SSID, PASS);

    // the lines below are for connecting to a WPA2 enterprise network 
    // (taken from the oficial wpa2_enterprise example from esp-idf)
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ID, strlen(EAP_ID)) );
    ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_UNAME, strlen(EAP_UNAME)) );
    ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASS, strlen(EAP_PASS)) );
    ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_enable() );
    WiFi.begin(SSID);

    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        Serial.print(".");
    }
    Serial.println();

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

WiFiClient espClient;
PubSubClient client(espClient);

void reconnectMQTTClient() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");

        if (client.connect(CLIENT_NAME.c_str())) {
            Serial.println("connected");
        }
        else {
            Serial.print("Retrying in 5 seconds - failed, rc=");
            Serial.println(client.state());
            
            delay(5000);
        }
    }
    client.subscribe(SERVER_COMMAND_TOPIC.c_str());
}
void clientCallback(char *topic, uint8_t *payload, unsigned int length)
{
    char buff[length + 1];
    for (int i = 0; i < length; i++)
    {
        buff[i] = (char)payload[i];
    }
    buff[length] = '\0';

    Serial.print("Message received:");
    Serial.println(buff);

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, buff);
    JsonObject obj = doc.as<JsonObject>();

    bool led_on = obj["led_on"];

    if (led_on)
        digitalWrite(PIN_R, HIGH);
    else
        digitalWrite(PIN_R, LOW);
}
void createMQTTClient() {
    client.setCallback(clientCallback);
    client.setServer(BROKER.c_str(), 1883);
    reconnectMQTTClient();
}

Adafruit_BME280 bme = Adafruit_BME280();

void setup() {
	Serial.begin(115200);

	while (!Serial)
		; // Wait for Serial to be ready

	delay(2000);

    Wire.begin(47, 48);
    bme.begin(0x76, &Wire);

    connectWiFi();
    createMQTTClient();
    pinMode(PIN_R, OUTPUT);
}

void loop() {
    reconnectMQTTClient();
    client.loop();

    float temp = bme.readTemperature();
    float humi = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F;
    Serial.printf("Temperature: %f°C, humidity: %f%, pressure: %f hPa \n", temp, humi, pres);

    DynamicJsonDocument doc(256);
    doc["temperature"] = temp;

    string telemetry;
    serializeJson(doc, telemetry);

    Serial.print("Sending telemetry ");
    Serial.println(telemetry.c_str());

    client.publish(CLIENT_TELEMETRY_TOPIC.c_str(), telemetry.c_str());
    
        if (temp < 25) {
        digitalWrite(PIN_R, HIGH);
        } else {
        digitalWrite(PIN_R, LOW);
        }
    delay(5000);
}
