#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 27
#define DHTTYPE DHT11
#define LEDPIN_POWER 17        // LED untuk indikator daya
#define LEDPIN_WIFI 18         // LED untuk indikator koneksi Wi-Fi
#define LEDPIN_KIRIMDATA 19    // LED untuk indikator pengiriman data

const char* ssid = "RIAN";
const char* password = "rianwelut";
// const char* websocket_server = "http://192.168.2.194:3000/data";  // Ganti sesuai alamat WebSocket server
const char* websocket_server = "http://192.168.0.189";
const uint16_t websocket_port = 3000; 

DHT_Unified dht(DHTPIN, DHTTYPE);
WebSocketsClient webSocket;

float t;
float h;
unsigned long previousMillis = 0;
const long interval = 60000;

// Fungsi untuk indikator pengiriman data
void blinkled() {
  digitalWrite(LEDPIN_KIRIMDATA, HIGH);  // Nyalakan LED
  delay(300);
  digitalWrite(LEDPIN_KIRIMDATA, LOW);   // Matikan LED
  delay(10);
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Set semua LED sebagai output
  pinMode(LEDPIN_POWER, OUTPUT);
  pinMode(LEDPIN_WIFI, OUTPUT);
  pinMode(LEDPIN_KIRIMDATA, OUTPUT);

  // Indikator daya alat hidup
  digitalWrite(LEDPIN_POWER, HIGH);  // Nyalakan LED daya

  // Koneksi ke Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LEDPIN_WIFI, HIGH);  // Blink LED WiFi selama proses koneksi
    delay(200);
    digitalWrite(LEDPIN_WIFI, LOW);
  }
  Serial.println("\nWiFi connected");
  digitalWrite(LEDPIN_WIFI, HIGH);  // LED WiFi tetap nyala setelah koneksi sukses

  setupWebSocket();
}

void setupWebSocket() {
  webSocket.begin(websocket_server, websocket_port, "/", "data");
  webSocket.onEvent(webSocketEvent);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket Connected");
      break;
    case WStype_TEXT:
      Serial.printf("WebSocket message: %s\n", payload);
      break;
  }
}

void getCalibratedData() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  t = event.temperature;
  dht.humidity().getEvent(&event);
  h = event.relative_humidity;
}

void sendDataSensor() {
  // Ambil data sensor yang sudah terkalibrasi
  getCalibratedData();

  // Buat JSON string dengan data temperature dan humidity
  String jsonData = "{\"temperature\": " + String(t) + ", \"humidity\": " + String(h) + "}";

  // Kirim data JSON melalui WebSocket
  webSocket.sendTXT(jsonData);
  Serial.println("Data sent via WebSocket: " + jsonData);

  // Indikator LED untuk menandakan data telah dikirim
  blinkled();
}

void loop() {
  // Proses WebSocket
  webSocket.loop();

  // Kirim data sensor setiap interval tertentu
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendDataSensor();
  }
}
