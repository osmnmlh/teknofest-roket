// DEĞİŞİKLİK: LoRa_E32 yerine LoRa_e32ttl kütüphanesi kullanıldı - tutarlılık için
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "LoRa_E32.h"

// Pin tanımları
#define GPS_RX 10
#define GPS_TX 11
#define LORA_RX 3
#define LORA_TX 2
#define LED1_PIN 6
#define LED2_PIN 7

// DEĞİŞİKLİK: LoRa e32ttl için M0 ve M1 pinleri eklendi
// #define M0 4
// #define M1 5

unsigned long lastLEDToggle = 0;
const unsigned long LED_INTERVAL = 500;
bool ledState = false;

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
SoftwareSerial loraSerial(LORA_RX, LORA_TX);
TinyGPSPlus gps;

// DEĞİŞİKLİK: LoRa_E32 yerine LoRa_e32ttl kullanıldı
LoRa_E32 e32ttl(&loraSerial, UART_BPS_RATE_9600);

float payload_gps_alt = 0.0f;
float payload_lat     = 0.0f;
float payload_lon     = 0.0f;

// DEĞİŞİKLİK: LoRa_e32ttl uyumluluğu için struct packed olarak tanımlandı
#pragma pack(push,1)
struct Signal {
  float payload_gps_alt;
  float payload_lat;
  float payload_lon;
};
#pragma pack(pop)

Signal data2;

unsigned long lastSend = 0;
unsigned long lastGPSRead = 0;
unsigned long lastStatusPrint = 0;

const unsigned long SEND_INTERVAL = 300;
const unsigned long GPS_READ_CYCLE = 3000;
const unsigned long STATUS_PRINT_INTERVAL = 5000;

bool gpsDataValid = false;
float lastValidLat = 1.0;
float lastValidLon = 1.0;
float lastValidAlt = 1.0;xxxxxxxxxxxxxxxxxxxxxxxxxxx

void setup() {
  // DEĞİŞİKLİK: M0 ve M1 pinleri eklendi - LoRa e32ttl için gerekli
  // pinMode(M0, OUTPUT);
  // pinMode(M1, OUTPUT);
  // digitalWrite(M0, LOW);
  // digitalWrite(M1, LOW);
  
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  Serial.begin(9600);
  gpsSerial.begin(9600);
  loraSerial.begin(9600);
  
  // DEĞİŞİKLİK: e32ttl başlatma metodu güncellendi
  e32ttl.begin();
  delay(500);
  
  Serial.println(F("=== GPS + LoRa e32ttl Verici Başladı ==="));
}

void loop() {
  // LED kontrolü (non-blocking)
  if (millis() - lastLEDToggle >= LED_INTERVAL) {
    lastLEDToggle = millis();
    ledState = !ledState;
    digitalWrite(LED1_PIN, ledState);
    digitalWrite(LED2_PIN, ledState);
  }

  // GPS veri okuma (non-blocking)
  readGPSData();

  // LoRa gönderim
  if (millis() - lastSend >= SEND_INTERVAL ) {
    sendLoRaData();
    lastSend = millis();
  }

  // Durum bilgisi
  if (!gpsDataValid && millis() - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
    printGPSStatus();
    lastStatusPrint = millis();
  }
}

void readGPSData() {
  if (millis() - lastGPSRead >= GPS_READ_CYCLE) {
    lastGPSRead = millis();
    
    gpsSerial.listen();// DEĞİŞİKLİK: SoftwareSerial listen() çağrısı kaldırıldı - e32ttl için gerekli değil
    
    // GPS verilerini oku
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        updateGPSData();
      }
    }
  }
}

void updateGPSData() {
  if (gps.location.isValid()) {
    gpsDataValid = true;
    lastValidLat = gps.location.lat();
    lastValidLon = gps.location.lng();
    lastValidAlt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  }
}

void sendLoRaData() {
  data2.payload_lat = lastValidLat;
  data2.payload_lon = lastValidLon;
  data2.payload_gps_alt = lastValidAlt;
  
  // DEĞİŞİKLİK: Ana aviyoniğe gönderim - address 0, addh 1, channel 23
  // Ana aviyonik bu kanaldan dinleyecek
  ResponseStatus rs = e32ttl.sendFixedMessage(0, 1, 18, &data2, sizeof(Signal));
  Serial.println(rs.getResponseDescription());

  Serial.println(F("\n=== LoRa e32ttl Gönderim ==="));
  Serial.print(F("Gönderim Durumu: ")); Serial.println(rs.code);
  Serial.print(F("Latitude  : ")); Serial.println(data2.payload_lat, 6);
  Serial.print(F("Longitude : ")); Serial.println(data2.payload_lon, 6);
  Serial.print(F("Altitude  : ")); Serial.print(data2.payload_gps_alt, 2); Serial.println(" m");
  Serial.print(F("Struct Size: ")); Serial.println(sizeof(Signal));
  Serial.println(F("======================="));
}

void printGPSStatus() {
  Serial.print(F("GPS Fix bekleniyor... Satellites: "));
  Serial.println(gps.satellites.value());
}

void printTwoDigits(int number) {
  if (number < 10) Serial.print('0');
  Serial.print(number);
}