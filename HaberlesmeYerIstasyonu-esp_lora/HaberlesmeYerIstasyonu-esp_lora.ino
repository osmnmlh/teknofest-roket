#include <Arduino.h>

// Lora Modül Pin Tanımlamaları
#define LORA_RX_PIN 17  // Lora'nın TXD'si G17'ye bağlı (ESP'nin RX'i)
#define LORA_TX_PIN 16  // Lora'nın RXD'si G16'ya bağlı (ESP'nin TX'i)
#define LORA_M1_PIN 5   // Lora M1 Pini
#define LORA_M0_PIN 4   // Lora M0 Pini

// STM32 ve ESP32 arasında kullanılacak ortak Telemetri Veri Yapısı
// Bellek hizalamasını kapatmak için 'packed' niteliğini kullanıyoruz
struct __attribute__((packed)) TelemetryData {
  float ms5611_altitude;     // MS5611 İrtifa
  float gps_latitude;        // GPS Enlem
  float gps_longitude;       // GPS Boylam
  float gps_altitude;        // GPS İrtifa
  float bno055_roll;         // BNO055 Roll (Yuvarlanma)
  float bno055_pitch;        // BNO055 Pitch (Yunuslama)
  float bno055_yaw;          // BNO055 Yaw (Sapma)
};

TelemetryData rocketData;

void setup() {
  // Bilgisayar (C# Arayüzü / Serial Monitör) ile iletişim
  Serial.begin(115200);
  
  // Lora E22 modülü ile iletişim (Baud: 9600, 8N1)
  // ESP32'nin 2. Donanımsal Seri Portunu kullanıyoruz.
  Serial2.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  // M0 ve M1 pinlerini çıkış olarak ayarla
  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);

  // Modülü Normal Moda (Mode 0) al
  // M0 = 0, M1 = 0 durumu şeffaf/sabit veri iletimi içindir
  digitalWrite(LORA_M0_PIN, LOW);
  digitalWrite(LORA_M1_PIN, LOW);

  Serial.println("Yer Istasyonu Baslatildi.");
  Serial.println("Lora E22-900T30D Dinleniyor...");
  
  // Modülün mod değiştirmesi için kısa bir süre bekle
  delay(100); 
}

void loop() {
  // Lora üzerinden gelen veri boyutu struct boyutumuza eşit veya büyükse
  if (Serial2.available() >= sizeof(TelemetryData)) {
    
    // Gelen byte'ları doğrudan struct içerisine kopyala
    Serial2.readBytes((uint8_t*)&rocketData, sizeof(TelemetryData));

    // Çöpe dönmüş veya eksik verileri temizlemek için kısa bir buffer temizliği (opsiyonel)
    while(Serial2.available() > 0) {
      Serial2.read();
    }

    // Verileri C# arayüzüne veya Serial Monitöre bas
    // C# tarafında ayrıştırmak için aralarına virgül koyarak yazdırabilirsin (CSV formatı)
    Serial.print("Data:");
    Serial.print(rocketData.ms5611_altitude); Serial.print(",");
    Serial.print(rocketData.gps_latitude, 6); Serial.print(","); // Hassasiyet için 6 hane
    Serial.print(rocketData.gps_longitude, 6); Serial.print(",");
    Serial.print(rocketData.gps_altitude); Serial.print(",");
    Serial.print(rocketData.bno055_roll); Serial.print(",");
    Serial.print(rocketData.bno055_pitch); Serial.print(",");
    Serial.println(rocketData.bno055_yaw);
  }
}