#include <WiFi.h>
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

const char* ssid = "RIAN";
const char* password = "rianwelut";

String URL_temperature = "https://mpquic.research-ai.my.id/api/device/temperature";
String URL_metana = "https://mpquic.research-ai.my.id/api/device/metana";
String URL_humidity = "https://mpquic.research-ai.my.id/api/device/humidity";
String URL_dioksida = "https://mpquic.research-ai.my.id/api/device/dioksida";
String URL_amonia = "https://mpquic.research-ai.my.id/api/device/amonia";

String ip_local = "192.168.100.3";
String URL_temperature_local = "http://" + ip_local + "/is4ac_local/temperature_data.php";
String URL_metana_local = "http://" + ip_local + "/is4ac_local/metana_data.php";
String URL_humidity_local = "http://" + ip_local + "/is4ac_local/humidity_data.php";
String URL_dioksida_local = "http://" + ip_local + "/is4ac_local/dioksida_data.php";
String URL_amonia_local = "http://" + ip_local + "/is4ac_local/amonia_data.php";

int id_alat_iot = 1;              
const int tipe_data = 3;          
const int metode_kirim_data = 1;  
const int config_server = 1;      

int t;
int h;
int mq135Value;
int mq4Value;
int mq137Value;
int ulang;
DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t event;

unsigned long previousMillis = 0;
const long interval = 60000;

// Calibration references
float referenceTemperature = 30.1;  // Reference temperature in °C
float referenceHumidity = 69.0;     // Reference humidity in %

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

  Serial.print("Calibrated Temperature: ");
  Serial.println(t);
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
  tampilkan_konfigurasi();
  delay(10);

  cek_sensor_dht();
  delay(10);
  cek_sensor_mq();

  delay(5);
  getDataSensor();
  delay(5);

  // Initial calibration
  dht.temperature().getEvent(&event);
  float initialTemperature = event.temperature;
  dht.humidity().getEvent(&event);
  float initialHumidity = event.relative_humidity;
  calibrateSensor(initialTemperature, initialHumidity);
}

void loop() {
  delay(10);
  getCalibratedData();
  sendDataSensor();
  delay(60000);
}

void cek_sensor_mq() {
  updateSensorMQ135BeforeGet();
  delay(10);
  updateSensorMQ4BeforeGet();
  delay(10);
}

void cek_sensor_dht() {
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  delay(10);
  dht.humidity().getSensor(&sensor);

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  Serial.print(F("Temperature: "));
  Serial.print(event.temperature);
  Serial.println(F("°C"));
  delay(10);

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  Serial.print(F("Humidity: "));
  Serial.print(event.relative_humidity);
  Serial.println(F("%"));
  delay(10);
}

void tampilkan_konfigurasi() {
  Serial.println("Current Device Configuration");

  if (tipe_data == 1) Serial.println("Using Sensor Data");
  if (tipe_data == 2) Serial.println("Using Dummy Data");
  if (tipe_data == 3) Serial.println("DHT sensor data, ammonia, dioxide dummy data");

  if (metode_kirim_data == 1) Serial.println("Data sent Automatically at regular intervals");
  if (metode_kirim_data == 2) Serial.println("Data sent manually using a trigger");

  if (config_server == 1) Serial.println("Using Public Server");
  if (config_server == 2) Serial.println("Using Local Server");

  Serial.println(id_alat_iot);
  delay(5);
}

void dataSensorToHTTP(int idalat, int nilaisensor, String URL) {
  getCalibratedData();
  delay(20);
  String postData = "id_alat=" + String(idalat) + "&nilai=" + String(nilaisensor);

  WiFiClientSecure client;
  client.setInsecure();  // Disable SSL certificate verification

  HTTPClient http;
  http.begin(client, URL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  auto httpCode = http.POST(postData);
  String payload = http.getString();
  Serial.println(URL);
  Serial.println(postData);
  Serial.println(payload);

  blinkled();
  http.end();
}

void blinkled() {
  analogWrite(LEDPIN_KIRIMDATA, 200);
  delay(300);
  analogWrite(LEDPIN_KIRIMDATA, 0);
  delay(10);
}

void updateSensorMQ137BeforeGet() {
  mq137Value = 0.75 * mq135Value + -0.635;
  Serial.print("NH3 | PPM: ");
  Serial.println(mq137Value);
  delay(10);
}

void updateSensorMQ135BeforeGet() {
  int analogValue = analogRead(MQ135PIN);
  float voltage = analogValue * (3.3 / 4095.0);
  float rs = (3.3 * 20.0 / voltage) - 20.0;
  mq135Value = 116.6020682 * pow(rs / 76.63, -2.769034857);
  Serial.print("CO2 | PPM: ");
  Serial.println(mq135Value);
  delay(10);
}

void updateSensorMQ4BeforeGet() {
  int analogValue = analogRead(MQ4PIN);
  float voltage = analogValue * (3.3 / 4095.0);
  float rs = (3.3 * 30.0 / voltage) - 30.0;
  mq4Value = 116.6020682 * pow(rs / 76.63, -2.769034857);
  Serial.print("Methane PPM: ");
  Serial.println(mq4Value);
  delay(10);
}

void getDataSensor() {
  if (tipe_data == 1) {
    dht.temperature().getEvent(&event);
    t = event.temperature;
    delay(10);
    dht.humidity().getEvent(&event);
    h = event.relative_humidity;
    delay(10);

    updateSensorMQ135BeforeGet();
    updateSensorMQ4BeforeGet();
    delay(5);
  }
  if (tipe_data == 2) {
    t = random(29, 31);
    h = random(71, 73);
    mq4Value = random(200, 500);
    mq135Value = random(400, 600);
    mq137Value = random(600, 800);
    delay(10);
  }
  if (tipe_data == 3) {
    dht.temperature().getEvent(&event);
    t = event.temperature;
    delay(10);
    dht.humidity().getEvent(&event);
    h = event.relative_humidity;
    delay(10);

    updateSensorMQ137BeforeGet();
    updateSensorMQ4BeforeGet();
    delay(5);
  }
}

void sendDataSensor() {
  Serial.print("Sending Temperature Data ");
  dataSensorToHTTP(id_alat_iot, t, URL_temperature);
  delay(20);

  Serial.print("Sending Humidity Data ");
  dataSensorToHTTP(id_alat_iot, h, URL_humidity);
  delay(20);
}
