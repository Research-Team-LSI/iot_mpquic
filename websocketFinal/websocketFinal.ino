#include <WiFi.h>
#include <time.h>
#include <WiFiClientSecure.h>  // For HTTPS support
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFiUdp.h>

#define DHTPIN 27       // Pin for the DHT-11 sensor
#define MQ135PIN 32     // Pin for the MQ-135 sensor
#define MQ4PIN 33       // Metana
#define MQ137PIN 35     // CO2
#define LEDindicator 2  // Built-in LED for ESP32
#define LEDPIN_POWER 17
#define LEDPIN_WIFI 18
#define LEDPIN_KIRIMDATA 19

#define DHTTYPE DHT11  // DHT 11

// NTP server details
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;  // Adjust for your timezone, e.g., 7 hours for UTC+7
const int   daylightOffset_sec = 0;
// wifi details
const char* ssid = "Redmi Note 12 Pro";
const char* password = "oktabrians";

String URL_temperature = "http://192.168.177.35:3000/data";

int id_alat_iot = 1;
const int tipe_data = 3;
const int metode_kirim_data = 1;
const int config_server = 1;

float t;  // Temperature
float h;  // Humidity
DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t event;

unsigned long previousMillis = 0;
const long interval = 60000;

// Calibration references
float referenceTemperature = 29.1;  // Reference temperature in °C
float referenceHumidity = 66;       // Reference humidity in %

float temperatureOffset = 0.0;
float humidityOffset = 0.0;

// Function to calibrate sensor readings
void calibrateSensor(float rawTemperature, float rawHumidity) {
  temperatureOffset = referenceTemperature - rawTemperature;
  humidityOffset = referenceHumidity - rawHumidity;
}

// Function to get calibrated sensor data
void getCalibratedData() {
  dht.temperature().getEvent(&event);
  float rawTemperature = event.temperature;
  dht.humidity().getEvent(&event);
  float rawHumidity = event.relative_humidity;

  // Apply calibration offsets
  t = rawTemperature + temperatureOffset;
  h = rawHumidity + humidityOffset;

  Serial.print("Raw Temperature: ");
  Serial.println(rawTemperature);
  Serial.print("Calibrated Temperature: ");
  Serial.println(t);

  Serial.print("Raw Humidity: ");
  Serial.println(rawHumidity);
  Serial.print("Calibrated Humidity: ");
  Serial.println(h);
}

void setup() {
  Serial.begin(115200);

  dht.begin();
  delay(5);

  pinMode(LEDindicator, OUTPUT);
  pinMode(LEDPIN_KIRIMDATA, OUTPUT);
  pinMode(LEDPIN_POWER, OUTPUT);
  pinMode(LEDPIN_WIFI, OUTPUT);
  pinMode(MQ135PIN, INPUT);
  delay(5);

  digitalWrite(LEDindicator, LOW);
  digitalWrite(LEDPIN_KIRIMDATA, LOW);
  digitalWrite(LEDPIN_POWER, LOW);
  digitalWrite(LEDPIN_WIFI, LOW);
  delay(5);
  digitalWrite(LEDPIN_POWER, HIGH);
  delay(5);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    analogWrite(LEDPIN_WIFI, 150);
    delay(200);
    analogWrite(LEDPIN_WIFI, 0);
  }
  Serial.println("WiFi connected");
  analogWrite(LEDPIN_WIFI, 150);
  delay(10);
  Serial.println(WiFi.localIP());

  // Initial calibration
  dht.temperature().getEvent(&event);
  float initialTemperature = event.temperature;
  dht.humidity().getEvent(&event);
  float initialHumidity = event.relative_humidity;
  calibrateSensor(initialTemperature, initialHumidity);

   // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void loop() {
  delay(10);
  getCalibratedData();
  sendDataSensor();
  delay(1000);
}

void sendDataSensor() {
  Serial.print("Sending Data ");
  dataSensorToHTTP(t, h, URL_temperature);
  delay(20);
}

String getFormattedTimestamp() {
  struct tm timeInfo;
  if (!getLocalTime(&timeInfo)) {
    Serial.println("Failed to obtain time");
    return "";
  }

  // Format waktu tanpa milidetik
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%d/%m/%Y, %H:%M:%S", &timeInfo);

  // Dapatkan milidetik tiga digit
  int milliseconds = millis() % 1000;
  String millisecondsStr = String(milliseconds);

  // Tambahkan padding jika milidetik kurang dari tiga digit
  if (milliseconds < 10) {
    millisecondsStr = "00" + millisecondsStr;
  } else if (milliseconds < 100) {
    millisecondsStr = "0" + millisecondsStr;
  }

  // Gabungkan timestamp dengan milidetik
  String formattedTimestamp = String(buffer) + ":" + millisecondsStr;
  return formattedTimestamp;
}


void dataSensorToHTTP(float nilaisensor, float nilaisensor2, String URL) {
  getCalibratedData();
  delay(20);
  
  // Get the current timestamp
  String timestamp = getFormattedTimestamp();

  // Construct JSON payload
  String postData = "{\"id_alat\": " + String(id_alat_iot) +
                    ", \"temperature\": " + String(t, 1) +
                    ", \"humidity\": " + String(h, 1) +
                    ", \"kirimdata\": \"" + timestamp + "\"}";

  WiFiClient client;
  HTTPClient http;
  http.begin(client, URL);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST(postData);

  if (httpCode > 0) {
    Serial.println("Data sent successfully!");
    Serial.println("Response code: " + String(httpCode));
    String payload = http.getString();
    Serial.println("Server response: " + payload);
    blinkled();
  } else {
    Serial.println("Error in sending data.");
    Serial.print("HTTP error code: ");
    Serial.println(httpCode);
  }

  http.end();
}


void blinkled() {
  analogWrite(LEDPIN_KIRIMDATA, 200);
  delay(300);
  analogWrite(LEDPIN_KIRIMDATA, 0);
  delay(10);
}
