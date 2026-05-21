#include <Wire.h>
#include "LoRa_E22.h"
#include <vector>

std::vector<uint8_t> receiveBuffer;
const size_t PACKET_SIZE = 78;

// --- LoRa E22 tanımı ---
#define M0 32 
#define M1 33
#define LORA_RX  27
#define LORA_TX  35

HardwareSerial ArfesaSerial(1);                          
LoRa_E22 e22(LORA_TX, LORA_RX, &ArfesaSerial, UART_BPS_RATE_9600);

// --- EK7 Paket Sabitleri ve yapı ---
#define PACKET_HEADER0 0xFF
#define PACKET_HEADER1 0xFF
#define PACKET_HEADER2 0x54
#define PACKET_HEADER3 0x52
#define PACKET_FOOTER0 0x0D
#define PACKET_FOOTER1 0x0A

// Checksum hesaplama fonksiyonu
static uint8_t compute_checksum(const uint8_t *data) {
    uint16_t sum = 0;
    for (size_t i = 4; i <= 74; ++i) sum += data[i];
    return (uint8_t)(sum % 256);
}

// Telemetri paketi yapısı
#pragma pack(push,1)
typedef struct {
    uint8_t hdr0, hdr1, hdr2, hdr3;
    uint8_t team_id;
    uint8_t packet_count;
    float alt_rel;
    float gps_alt;
    float lat;
    float lon;
    float payload_gps_alt;
    float payload_lat;
    float payload_lon;
    float stage_gps_alt;
    float stage_lat;
    float stage_lon;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float angle;
    uint8_t status;
    uint8_t checksum;
    uint8_t ftr0, ftr1;
} TelemetryPacket;
#pragma pack(pop)

// Payload GPS verisi için yapı (Gönderici kodunuzla uyumlu)
#pragma pack(push,1)
struct EkHaberlesme {
    float ek_gps_alt;
    float ek_lat;
    float ek_lon;
};
#pragma pack(pop)

_Static_assert(sizeof(TelemetryPacket) == 78, "TelemetryPacket must be 78 bytes");

// İstatistik değişkenleri
unsigned long totalPackets = 0;
unsigned long validPackets = 0;
unsigned long checksumErrors = 0;
unsigned long headerErrors = 0;
uint8_t lastPacketCount = 0;
unsigned long missedPackets = 0;
unsigned long lastReceiveTime = 0;

// Payload GPS verileri
EkHaberlesme latestPayloadData = {0.0f, 0.0f, 0.0f};
bool payloadDataReceived = false;
unsigned long lastPayloadReceiveTime = 0;
unsigned long payloadDataCount = 0;

// Fonksiyon bildirimleri
void printTelemetryData(const TelemetryPacket *pkt);
void printStatistics();
void updatePacketLoss(uint8_t currentPacketCount);
bool validatePacket(const uint8_t *data, size_t len);
void processPayloadData(const uint8_t* data, size_t len);
void printPayloadData();

// Paket doğrulama fonksiyonu
bool validatePacket(const uint8_t *data, size_t len) {
    if (len != 78) {
        Serial.println("Hatalı paket boyutu!");
        return false;
    }
    
    // Header kontrolü
    if (data[0] != PACKET_HEADER0 || data[1] != PACKET_HEADER1 || 
        data[2] != PACKET_HEADER2 || data[3] != PACKET_HEADER3) {
        Serial.println("Hatalı header!");
        headerErrors++;
        return false;
    }
      
    // Footer kontrolü
    if (data[76] != PACKET_FOOTER0 || data[77] != PACKET_FOOTER1) {
        Serial.println("Hatalı footer!");
        return false;
    }
    
    // Checksum kontrolü
    uint8_t calculated_checksum = compute_checksum(data);
    if (data[75] != calculated_checksum) {
        Serial.printf("Checksum hatası! Hesaplanan: 0x%02X, Alınan: 0x%02X\n", 
                     calculated_checksum, data[75]);
        checksumErrors++;
        return false;
    }
    
    return true;
}

// Payload GPS verilerini işle
void processPayloadData(const uint8_t* data, size_t len) {
    if (len == sizeof(EkHaberlesme)) {
        // Doğru boyutta payload verisi
        memcpy(&latestPayloadData, data, sizeof(EkHaberlesme));
        payloadDataReceived = true;
        lastPayloadReceiveTime = millis();
        payloadDataCount++;
        
        Serial.println("\n========== PAYLOAD GPS DATA ==========");
        Serial.printf("Payload Data Count: %lu\n", payloadDataCount);
        Serial.printf("Payload GPS Alt: %.2f m\n", latestPayloadData.ek_gps_alt);
        Serial.printf("Payload Lat: %.6f°\n", latestPayloadData.ek_lat);
        Serial.printf("Payload Lon: %.6f°\n", latestPayloadData.ek_lon);
        Serial.println("=====================================\n");
    } else {
        Serial.printf("Payload veri boyutu hatalı! Alınan: %d, Beklenen: %d\n", 
                     len, sizeof(EkHaberlesme));
    }
}

// Paket kayıp sayısını hesapla
void updatePacketLoss(uint8_t currentPacketCount) {
    if (totalPackets > 0) {  // İlk paket değilse
        uint8_t expected = (lastPacketCount + 1) % 256;
        if (currentPacketCount != expected) {
            if (currentPacketCount > expected) {
                missedPackets += (currentPacketCount - expected);
            } else {
                // Overflow durumu (255'den 0'a geçiş)
                missedPackets += (256 - expected + currentPacketCount);
            }
        }
    }
    lastPacketCount = currentPacketCount;
}

// Telemetri verisini yazdır (payload verisiyle birleştirilmiş)
void printTelemetryData(const TelemetryPacket *pkt) {
    Serial.println("\n========== TELEMETRY DATA ==========");
    Serial.printf("Team ID: %d\n", pkt->team_id);
    Serial.printf("Packet Count: %d\n", pkt->packet_count);
    Serial.printf("Relative Altitude: %.2f m\n", pkt->alt_rel);
    Serial.printf("GPS Altitude: %.2f m\n", pkt->gps_alt);
    Serial.printf("Latitude: %.6f°\n", pkt->lat);
    Serial.printf("Longitude: %.6f°\n", pkt->lon);
    
    Serial.println("\n--- PAYLOAD DATA (from Telemetry) ---");
    Serial.printf("Payload GPS Alt: %.2f m\n", pkt->payload_gps_alt);
    Serial.printf("Payload Lat: %.6f°\n", pkt->payload_lat);
    Serial.printf("Payload Lon: %.6f°\n", pkt->payload_lon);
    
    // En son alınan gerçek payload verisiyle karşılaştırma
    if (payloadDataReceived) {
        Serial.println("\n--- PAYLOAD DATA (Real GPS from Nano) ---");
        Serial.printf("Real Payload GPS Alt: %.2f m\n", latestPayloadData.ek_gps_alt);
        Serial.printf("Real Payload Lat: %.6f°\n", latestPayloadData.ek_lat);
        Serial.printf("Real Payload Lon: %.6f°\n", latestPayloadData.ek_lon);
        
        unsigned long payloadAge = (millis() - lastPayloadReceiveTime) / 1000;
        Serial.printf("Payload Data Age: %lu seconds\n", payloadAge);
    } else {
        Serial.println("\n--- PAYLOAD DATA (Real GPS) ---");
        Serial.println("No real payload GPS data received yet");
    }
    
    Serial.println("\n--- STAGE DATA ---");
    Serial.printf("Stage GPS Alt: %.2f m\n", pkt->stage_gps_alt);
    Serial.printf("Stage Lat: %.6f°\n", pkt->stage_lat);
    Serial.printf("Stage Lon: %.6f°\n", pkt->stage_lon);
    
    Serial.println("\n--- IMU DATA ---");
    Serial.printf("Gyro X: %.4f °/s\n", pkt->gyro_x);
    Serial.printf("Gyro Y: %.4f °/s\n", pkt->gyro_y);
    Serial.printf("Gyro Z: %.4f °/s\n", pkt->gyro_z);
    Serial.printf("Acc X: %.4f g\n", pkt->acc_x);
    Serial.printf("Acc Y: %.4f g\n", pkt->acc_y);
    Serial.printf("Acc Z: %.4f g\n", pkt->acc_z);
    Serial.printf("Pitch Angle: %.2f°\n", pkt->angle);
    Serial.printf("Status: 0x%02X\n", pkt->status);
    
    Serial.println("====================================");
}

// İstatistikleri yazdır
void printStatistics() {
    float successRate = (totalPackets > 0) ? (float)validPackets / totalPackets * 100.0f : 0;
    float packetLossRate = (totalPackets + missedPackets > 0) ? 
                          (float)missedPackets / (totalPackets + missedPackets) * 100.0f : 0;
    
    Serial.println("\n========== STATISTICS ==========");
    Serial.printf("Total Telemetry Packets: %lu\n", totalPackets);
    Serial.printf("Valid Telemetry Packets: %lu\n", validPackets);
    Serial.printf("Telemetry Success Rate: %.2f%%\n", successRate);
    Serial.printf("Header Errors: %lu\n", headerErrors);
    Serial.printf("Checksum Errors: %lu\n", checksumErrors);
    Serial.printf("Missed Packets: %lu\n", missedPackets);
    Serial.printf("Packet Loss Rate: %.2f%%\n", packetLossRate);
    Serial.printf("Last Packet Count: %d\n", lastPacketCount);
    
    // Payload istatistikleri
    Serial.printf("Payload GPS Packets: %lu\n", payloadDataCount);
    if (payloadDataReceived) {
        unsigned long payloadAge = (millis() - lastPayloadReceiveTime) / 1000;
        Serial.printf("Last Payload Age: %lu seconds\n", payloadAge);
    }
    
    if (lastReceiveTime > 0) {
        unsigned long timeSinceLastPacket = (millis() - lastReceiveTime) / 1000;
        Serial.printf("Time Since Last Data: %lu seconds\n", timeSinceLastPacket);
    }
    Serial.println("===============================\n");
}

void setup() {
    // LoRa pinlerini ayarla
    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);
    digitalWrite(M0, LOW);
    digitalWrite(M1, LOW);
    
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("========================================");
    Serial.println("   DUAL RECEIVER - EK7 & PAYLOAD GPS   ");
    Serial.println("========================================");
    
    // LoRa E22 başlat
    e22.begin();
    delay(500);
    
    Serial.println("LoRa E22 initialized successfully!");
    Serial.println("Listening for:");
    Serial.println("  - EK7 Telemetry packets (78 bytes)");
    Serial.println("  - Payload GPS data (12 bytes)");
    Serial.println("Waiting for data...\n");
}

void loop() {
    // LoRa mesaj alma
    if (e22.available() > 0) {
        ResponseContainer rc = e22.receiveMessage();
        if (rc.status.code == 1) {
            lastReceiveTime = millis();
            
            // Gelen verinin boyutuna göre işlem yap
            size_t dataLen = rc.data.length();
            
            if (dataLen == sizeof(EkHaberlesme)) {
                // Payload GPS verisi (12 byte)
                Serial.printf("Payload GPS verisi alındı (%d bytes)\n", dataLen);
                processPayloadData((const uint8_t*)rc.data.c_str(), dataLen);
                
            } else {
                // EK7 telemetri paketi olabilir - tampona ekle
                Serial.printf("Telemetri verisi alındı (%d bytes), tampona ekleniyor\n", dataLen);
                const char* data = rc.data.c_str();
                receiveBuffer.insert(receiveBuffer.end(), data, data + dataLen);
            }
            
        } else {
            Serial.printf("LoRa receive error: %d\n", rc.status.code);
        }
    }

    // EK7 telemetri paketlerini işle
    while (receiveBuffer.size() >= PACKET_SIZE) {
        // Tamponun başında paket başlangıç kodunu (header) ara
        bool packetFound = false;
        size_t headerPos = 0;

        for (size_t i = 0; i <= receiveBuffer.size() - 4; ++i) {
            if (receiveBuffer[i]   == PACKET_HEADER0 &&
                receiveBuffer[i+1] == PACKET_HEADER1 &&
                receiveBuffer[i+2] == PACKET_HEADER2 &&
                receiveBuffer[i+3] == PACKET_HEADER3)
            {
                // Header bulundu, bu pozisyondan itibaren yeterli veri var mı?
                if (receiveBuffer.size() >= i + PACKET_SIZE) {
                    headerPos = i;
                    packetFound = true;

                    // Header'dan önceki bozuk verileri temizle
                    if (headerPos > 0) {
                        Serial.printf("Tamponun başındaki %d byte'lık bozuk veri atılıyor.\n", headerPos);
                        receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + headerPos);
                    }
                    break;
                }
            }
        }

        if (packetFound) {
            totalPackets++; // İstatistik için sayaç
            Serial.printf("\n[TELEMETRY PACKET #%lu] Aday paket bulundu, doğrulanıyor...\n", totalPackets);

            // Aday paketi doğrula
            uint8_t* packetData = receiveBuffer.data();
            if (validatePacket(packetData, PACKET_SIZE)) {
                validPackets++;

                TelemetryPacket* pkt = (TelemetryPacket*)packetData;
                updatePacketLoss(pkt->packet_count);
                printTelemetryData(pkt);

            } else {
                Serial.println("Geçersiz telemetri paketi!");
                // Hatalı paketi atla (sadece header'ı silerek bir sonraki denemede senkronizasyonu bulmayı dene)
                receiveBuffer.erase(receiveBuffer.begin());
                continue; // Döngüye baştan başla
            }

            // İşlenen paketi tampondan sil
            receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + PACKET_SIZE);

        } else {
            // Tamponda tam bir paket bulunamadı, döngüden çık
            break;
        }
    }

    // Periyodik istatistik yazdırma
    static unsigned long lastStatsTime = 0;
    if (millis() - lastStatsTime > 10000) { // Her 10 saniyede bir kontrol et
        lastStatsTime = millis();
        if (validPackets > 0 || payloadDataReceived) { 
            // Eğer en az bir geçerli veri alındıysa istatistikleri yazdır
            printStatistics();
        } else if (totalPackets == 0 && payloadDataCount == 0) { 
            // Hiç veri gelmediyse bekleme mesajı yazdır
            Serial.println("Ana aviyonik ve payload GPS verisi bekleniyor...");
        }
    }
    
    delay(10);  // CPU yükünü azaltmak için
}