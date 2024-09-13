#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "MAX30100_PulseOximeter.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define WiFi and MQTT Credentials
const char *ssid = "HUAWEI Y8s";
const char *pass = "toka1234";
const char *mqtt_server = "a3d4f951f9b6453ca2fad9d2c0d38505.s1.eu.hivemq.cloud";
const char *mqtt_username = "tokaAB";
const char *mqtt_password = "Stemer2022@";
const int mqtt_port = 8883;

// Sensor pins
#define gasensor 34
#define buzzer 15
#define gas_THRESHOLD 1500
#define TEMP_THRESHOLD 38.0
#define HUM_THRESHOLD 29.0
#define micpin 2
#define fan 4
#define DHTPIN 19
#define DHTTYPE DHT11
#define led 18
#define I2C_SDA 21
#define I2C_SCL 22

// Sensor instances
DHT dht(DHTPIN, DHTTYPE);
PulseOximeter pox;

// WiFi and MQTT client setup
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Task handles
TaskHandle_t GasTaskHandle, TempHumTaskHandle, HeartRateTaskHandle, micTaskHandle;

// MQTT message buffer
#define MSG_BUFFER_SIZE (500)
char msg[MSG_BUFFER_SIZE];

// HiveMQ Cloud Certificate
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// Function prototypes for FreeRTOS tasks
void GasSensorTask(void *pvParameters);
void TempHumidityTask(void *pvParameters);
void HeartRateTask(void *pvParameters);
void micTask(void *pvParameters);

void setup_wifi()

{
    Serial.println("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected!");
}
void callback(char *topic, byte *payload, unsigned int length)
{
    // Serial.print("Message arrived [");
    // Serial.print(topic);
    // Serial.print("] ");
    // for (int i = 0; i < length; i++)
    // {
    //   Serial.print((char)payload[i]);
    // }
    // Serial.println();
    Serial.print("Message received on topic: ");
    Serial.println(topic);

    // Convert payload to a string
    String message = "";
    for (int i = 0; i < length; i++)
    {
        message += (char)payload[i];
    }

    Serial.print("Message: ");
    Serial.println(message);

    // Display the message on the LCD

    if (message.length() > 16)
    {
        // Second line
        Serial.print(message);
    }
    else
    {
        // If the message fits in one line, display it in one line

        Serial.print(message);
        delay(500);
    }

    // Control the LED based on the message
    if (message.equals("HIGH"))
    {
        digitalWrite(led, HIGH); // Turn on LED
        Serial.println("LED ON");
    }
    else if (message.equals("LOW"))
    {
        digitalWrite(led, LOW); // Turn off LED
        Serial.println("LED OFF");
    }
    if (message.equals(0)){
      digitalWrite(buzzer,0);
    }
}

void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqtt_username, mqtt_password))
        {
            Serial.println("connected");
            client.subscribe("Flutter/to/ESP");
            client.subscribe("Flutter/buzzer/to/ESP");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            delay(5000);
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // Initialize MAX30100
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!pox.begin())
    {
        Serial.println("Failed to initialize MAX30100.");
        while (1)
            ; // Loop indefinitely if sensor initialization fails
    }
    pox.setOnBeatDetectedCallback([]()
                                  { Serial.println("Beat detected!"); });

    setup_wifi();
    espClient.setCACert(root_ca); // Make sure this is set before connecting to MQTT
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    // Initialize sensor pins
    pinMode(gasensor, INPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(micpin, INPUT);
    pinMode(fan, OUTPUT);
    pinMode(led, OUTPUT);
    digitalWrite(fan,LOW);
    dht.begin();

    // Create FreeRTOS tasks
    xTaskCreate(GasSensorTask, "GasSensorTask", 10000, NULL, 1, &GasTaskHandle);
    xTaskCreate(TempHumidityTask, "TempHumidityTask", 10000, NULL, 1, &TempHumTaskHandle);
    xTaskCreate(micTask, "micTask", 10000, NULL, 1, &micTaskHandle);
    xTaskCreate(HeartRateTask, "HeartRateTask", 10000, NULL, 1, &HeartRateTaskHandle);
}

void loop()
{
    pox.update();

    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
}

// FreeRTOS Task for Gas Sensor
void GasSensorTask(void *pvParameters)
{
    int buzzerstate = 0;
    while (1)
    {
        int gasValue = analogRead(gasensor);
        if (gasValue >= gas_THRESHOLD)
        {
            digitalWrite(buzzer, HIGH);
            buzzerstate = 1;
        }
        else
        {
            digitalWrite(buzzer, LOW);
            buzzerstate = 0;
        }

        snprintf(msg, MSG_BUFFER_SIZE, "%d", gasValue);
        client.publish("ESP/gas/to/Flutter", msg);

        snprintf(msg, MSG_BUFFER_SIZE, "%d", buzzerstate);
        client.publish("ESP/buzzer/to/Flutter", msg);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds
    }
}

// FreeRTOS Task for Temperature and Humidity Sensor
void TempHumidityTask(void *pvParameters)
{
    while (1)
    {
        double Temperature = dht.readTemperature();
        double Humidity = dht.readHumidity();
        int fanstate = 0;

        if (Temperature >= TEMP_THRESHOLD || Humidity >= HUM_THRESHOLD)
        {
            digitalWrite(fan, HIGH);
            fanstate = 1;
        }
        else
        {
            digitalWrite(fan, LOW);
            fanstate = 0;
        }

        snprintf(msg, MSG_BUFFER_SIZE, "%.1f", Temperature);
        client.publish("ESP/temp/to/Flutter", msg);

        snprintf(msg, MSG_BUFFER_SIZE, "%.1f", Humidity);
        client.publish("humidity/dht/to/Flutter", msg);

        snprintf(msg, MSG_BUFFER_SIZE, "%d", fanstate);
        client.publish("ESP/fan/to/Flutter", msg);

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds
    }
}

void micTask(void *pvParameters)
{
    while (1)
    {
        boolean micreading;
        micreading = digitalRead(micpin);
        int micstate = 0;

        if (micreading == HIGH)
        {
            micstate = 1;
        }
        else
        {
            micstate = 0;
        }

        snprintf(msg, MSG_BUFFER_SIZE, "%d", micstate);
        client.publish("ESP/mic/to/Flutter", msg);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds
    }
}
// FreeRTOS Task for Heart Rate Sensor
void HeartRateTask(void *pvParameters)
{
    uint32_t lastReportTime = 0;
    while (1)
    {
        pox.update();
        if (millis() - lastReportTime > 1000)
        {
            double heartRate = pox.getHeartRate();
            double spO2 = pox.getSpO2();

            // Add debug output
            Serial.print("Heart Rate: ");
            Serial.println(pox.getHeartRate());
            Serial.print("SpO2: ");
            Serial.println(pox.getSpO2());

            snprintf(msg, MSG_BUFFER_SIZE, "%.1f", heartRate);
            client.publish("ESP/heartrate/to/Flutter", msg);

            snprintf(msg, MSG_BUFFER_SIZE, "%.1f", spO2);
            client.publish("ESP/spO2/to/Flutter", msg);
            lastReportTime = millis();
        }
    }
}

