#include <Wire.h>
#include <MS5611.h>
#include <math.h>
#include <TinyGPSPlus.h>   
#include "LoRa_E22.h"


// ——————————————————————————————————————————————————
// 1) Pin & sabit tanımları
// ——————————————————————————————————————————————————
#define MPU6500_ADDR    0x68
#define ACCEL_XOUT_H    0x3B
#define PWR_MGMT_1      0x6B

#define GPS_RX_PIN      16
#define GPS_TX_PIN      17

#define M0              32 
#define M1              33
#define LORA_RX         27
#define LORA_TX         35

#define TEAM_ID         1

// ——————————————————————————————————————————————————
// 2) Filtre & sensör sabitleri (configuration)
// ——————————————————————————————————————————————————
const float ACCEL_SENSITIVITY = 16384.0;
const float GYRO_SENSITIVITY  = 131.0;
const float alpha = 0.98;
const float KQ = 0.01;
const float KR = 0.1;


// ——————————————————————————————————————————————————
// 3) Zamanlama & Global Durum
// ——————————————————————————————————————————————————
const long interval = 250;
unsigned long prevMs = 0;

float acc_offsetX=0, acc_offsetY=0, acc_offsetZ=0;
float gyro_offsetX=0, gyro_offsetY=0, gyro_offsetZ=0;
float baselineAltitude = 0, kalman_est = 0, kalman_err = 1;
double baselinePitch = 0;

int16_t ax_raw, ay_raw, az_raw, gx_raw;
float accel_x, accel_y, accel_z, gyro_x;


float payload_gps_alt = 0.0f;
float payload_lat     = 0.0f;
float payload_lon     = 0.0f;

// Signal yapısı (görev yükü Arduino’dan gelecek)
struct Signal {
  float payload_gps_alt;
  float payload_lat;
  float payload_lon;
};

// ——————————————————————————————————————————————————
// 4) Paket Yapısı Sabitleri
// ——————————————————————————————————————————————————
#define PACKET_HEADER0 0xFF
#define PACKET_HEADER1 0xFF
#define PACKET_HEADER2 0x54
#define PACKET_HEADER3 0x52
#define PACKET_FOOTER0 0x0D
#define PACKET_FOOTER1 0x0A

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

_Static_assert(sizeof(TelemetryPacket) == 78, "TelemetryPacket must be 78 bytes");
uint8_t packetCount = 0;

// ——————————————————————————————————————————————————
// 5) Nesne Tanımları
// ——————————————————————————————————————————————————
MS5611          ms5611(MS5611_DEFAULT_ADDRESS, &Wire);
TwoWire         I2C_MPU6500 = TwoWire(1);
TinyGPSPlus     gps;
HardwareSerial  ArfesaSerial(1);
LoRa_E22        e22(LORA_TX, LORA_RX, &ArfesaSerial, UART_BPS_RATE_9600);

// ——————————————————————————————————————————————————
// 6) Yardımcı Fonksiyonlar
// ——————————————————————————————————————————————————
static uint8_t compute_checksum(const uint8_t *data) {
    uint16_t sum = 0;
    for (size_t i = 4; i <= 74; ++i) sum += data[i];
    return (uint8_t)(sum % 256);
}




void preparePacket_EK7(uint8_t *buf,
                      uint8_t team_id,
                      uint8_t packet_count,
                      float alt_rel,
                      float gps_alt,
                      float lat,
                      float lon,
                      float gyro_x,
                      float gyro_y,
                      float gyro_z,
                      float acc_x,
                      float acc_y,
                      float acc_z,
                      float angle,
                      uint8_t status)
{
    TelemetryPacket pkt;
    // Header
    pkt.hdr0 = PACKET_HEADER0;
    pkt.hdr1 = PACKET_HEADER1;
    pkt.hdr2 = PACKET_HEADER2;
    pkt.hdr3 = PACKET_HEADER3;

    pkt.team_id      = team_id;
    pkt.packet_count = packet_count;
    // Core veriler
    pkt.alt_rel = alt_rel;
    pkt.gps_alt = gps_alt;
    pkt.lat     = lat;
    pkt.lon     = lon;
    // Payload & stage: burada aynısını kullanıyoruz
    // Payload & stage: görev yükü Arduino’dan aldığımız verileri kullan
    pkt.payload_gps_alt = payload_gps_alt;
    pkt.payload_lat     = payload_lat;
    pkt.payload_lon     = payload_lon;

    pkt.stage_gps_alt   = 0;
    pkt.stage_lat       = 0;
    pkt.stage_lon       = 0;
    // IMU
    pkt.gyro_x = gyro_x;
    pkt.gyro_y = gyro_y;
    pkt.gyro_z = gyro_z;
    pkt.acc_x  = acc_x;
    pkt.acc_y  = acc_y;
    pkt.acc_z  = acc_z;
    pkt.angle  = angle;
    pkt.status = status;
    // Copy ve checksum
    memcpy(buf, &pkt, sizeof(pkt));
    buf[75] = compute_checksum(buf);
    // Footer
    buf[76] = PACKET_FOOTER0;
    buf[77]   = PACKET_FOOTER1;
}

float angleY=0, angleY_acc=0;
unsigned long last_time_mpu=0;

// Kalman filtre değişkenleri


// Pitch baseline

// Zamanlama



void calibrateMPU() {
  long ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  for(int i=0;i<500;i++){
    I2C_MPU6500.beginTransmission(MPU6500_ADDR);
    I2C_MPU6500.write(ACCEL_XOUT_H);
    I2C_MPU6500.endTransmission(false);
    I2C_MPU6500.requestFrom(MPU6500_ADDR,14,true);
    ax += (I2C_MPU6500.read()<<8 | I2C_MPU6500.read());
    ay += (I2C_MPU6500.read()<<8 | I2C_MPU6500.read());
    az += (I2C_MPU6500.read()<<8 | I2C_MPU6500.read());
    I2C_MPU6500.read(); I2C_MPU6500.read();
    gx += (I2C_MPU6500.read()<<8 | I2C_MPU6500.read());
    gy += (I2C_MPU6500.read()<<8 | I2C_MPU6500.read());
    gz += (I2C_MPU6500.read()<<8 | I2C_MPU6500.read());
    delay(2);
  }
  acc_offsetX = ax/500.0;
  acc_offsetY = ay/500.0;
  acc_offsetZ = az/500.0 - ACCEL_SENSITIVITY;
  gyro_offsetX= gx/500.0;
  gyro_offsetY= gy/500.0;
  gyro_offsetZ= gz/500.0;
}

void setup() {
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, 0);

  Serial.begin(115200);
  //while(!Serial);
  
  Wire.begin(21, 22);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS hazır");
  
  if(!ms5611.begin()){ Serial.println("MS5611 bulunamadi!"); while(1); }
  ms5611.setOversampling(OSR_ULTRA_HIGH);
  ms5611.read();
  float p0 = ms5611.getPressure()/100.0f;
  baselineAltitude = 44330.0f * (1.0f - pow(p0/1013.25f,0.1903f));
  kalman_est = baselineAltitude;

  I2C_MPU6500.begin(18,19);
  I2C_MPU6500.beginTransmission(MPU6500_ADDR);
  I2C_MPU6500.write(PWR_MGMT_1);
  I2C_MPU6500.write(0);
  I2C_MPU6500.endTransmission(true);
  calibrateMPU();
  last_time_mpu = micros();

  // İlk açı hesaplama
  I2C_MPU6500.beginTransmission(MPU6500_ADDR);
  I2C_MPU6500.write(ACCEL_XOUT_H);
  I2C_MPU6500.endTransmission(false);
  I2C_MPU6500.requestFrom(MPU6500_ADDR,14,true);
  ax_raw = (I2C_MPU6500.read()<<8 | I2C_MPU6500.read()) - acc_offsetX;
  ay_raw = (I2C_MPU6500.read()<<8 | I2C_MPU6500.read()) - acc_offsetY;
  az_raw = (I2C_MPU6500.read()<<8 | I2C_MPU6500.read()) - acc_offsetZ;
  I2C_MPU6500.read(); I2C_MPU6500.read();
  gx_raw = (I2C_MPU6500.read()<<8 | I2C_MPU6500.read()) - gyro_offsetX;
  accel_x = ax_raw/ACCEL_SENSITIVITY;
  accel_y = ay_raw/ACCEL_SENSITIVITY;
  accel_z = az_raw/ACCEL_SENSITIVITY;
  gyro_x  = gx_raw/GYRO_SENSITIVITY;
  angleY_acc = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z))*180/PI;
  angleY = angleY_acc;
  baselinePitch = angleY;

  
  e22.begin();
  delay(200);
  Serial.println("== ESP32 Gönderici Hazir ==");
}

void loop() {
    if (e22.available() >= sizeof(Signal)) {
      auto r = e22.receiveMessage(sizeof(Signal)); 
      // r.data void* pointer olduğu için direkt cast:
      Signal *s = (Signal*)r.data;
      payload_gps_alt = s->payload_gps_alt;
      payload_lat     = s->payload_lat;
      payload_lon     = s->payload_lon;
      r.close();
    }


    // GPS okumalarını işle
    while (Serial2.available()) {
      char c = Serial2.read();
      gps.encode(c);
  }

    // Zaman aralığı kontrolü
    if (millis() - prevMs < interval) return;
    prevMs = millis();

    // --- Basınç sensörü ve Kalman filtresi ile irtifa ---
    ms5611.read();
    float pressure = ms5611.getPressure() / 100.0f;
    float rawAltitude = 44330.0f * (1.0f - pow(pressure / 1013.25f, 0.1903f));
    kalman_err += KQ;
    float K = kalman_err / (kalman_err + KR);
    kalman_est += K * (rawAltitude - kalman_est);
    kalman_err *= (1 - K);
    float relAlt = kalman_est - baselineAltitude;

    // --- MPU9250 ile ivme ve açı hesabı (Complementary Filter) ---
    I2C_MPU6500.beginTransmission(MPU6500_ADDR);
    I2C_MPU6500.write(ACCEL_XOUT_H);
    I2C_MPU6500.endTransmission(false);
    I2C_MPU6500.requestFrom(MPU6500_ADDR, 14, true);
    ax_raw = (I2C_MPU6500.read() << 8 | I2C_MPU6500.read()) - acc_offsetX;
    ay_raw = (I2C_MPU6500.read() << 8 | I2C_MPU6500.read()) - acc_offsetY;
    az_raw = (I2C_MPU6500.read() << 8 | I2C_MPU6500.read()) - acc_offsetZ;
    // Temperature bytes
    I2C_MPU6500.read(); I2C_MPU6500.read();
    // Gyro X, Y, Z
    gx_raw = (I2C_MPU6500.read() << 8 | I2C_MPU6500.read()) - gyro_offsetX;
    int16_t gy_raw = (I2C_MPU6500.read() << 8 | I2C_MPU6500.read()) - gyro_offsetY;
    int16_t gz_raw = (I2C_MPU6500.read() << 8 | I2C_MPU6500.read()) - gyro_offsetZ;

    accel_x = ax_raw / ACCEL_SENSITIVITY;
    accel_y = ay_raw / ACCEL_SENSITIVITY;
    accel_z = az_raw / ACCEL_SENSITIVITY;
    gyro_x  = gx_raw / GYRO_SENSITIVITY;
    float gyro_y = gy_raw / GYRO_SENSITIVITY;
    float gyro_z = gz_raw / GYRO_SENSITIVITY;

    unsigned long now = micros();
    float dt = (now - last_time_mpu) / 1e6;
    last_time_mpu = now;

    angleY_acc = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / PI;
    angleY     = alpha * (angleY + gyro_x * dt) + (1 - alpha) * angleY_acc;
    float relPitch = angleY - baselinePitch;

    // --- GPS koordinatları ---
// --- GPS koordinatları ---
// Gerçek GPS verilerini okumak yerine sahte konum gönderelim
float lat = 38.708000;   // Sahte enlem
float lon = 35.518000;   // Sahte boylam
float gps_alt = 1070.0f; // Sahte irtifa

 

    // --- EK7 paketini hazırla ve gönder ---
    uint8_t buf[78];
    preparePacket_EK7(buf,
                      TEAM_ID,
                      packetCount++,
                      relAlt,
                      gps_alt,
                      lat,
                      lon,
                      gyro_x,
                      gyro_y,
                      gyro_z,
                      accel_x,
                      accel_y,
                      accel_z,
                      relPitch,
                      0x00);
    
    
    ResponseStatus rs = e22.sendFixedMessage(0, 2, 18, buf, sizeof(buf));

    // --- Debug çıktıları ---
    Serial.print("Header: ");
    for (int i = 0; i < 4; i++) { Serial.print(buf[i], HEX); Serial.print(" "); }
    Serial.println();
    Serial.print("TeamID: "); Serial.println(buf[4], DEC);
    Serial.print("PacketCount: "); Serial.println(buf[5], DEC);
    Serial.print("RelAlt: "); Serial.print(relAlt, 2); Serial.println(" m");
    Serial.print("GPS Alt: "); Serial.print(gps.altitude.meters(), 2); Serial.println(" m");
    Serial.print("Latitude: "); Serial.println(lat, 6);
    Serial.print("Longitude: "); Serial.println(lon, 6);
    Serial.print("Alinan Payload Lat: "); Serial.println(payload_lat, 6);
    Serial.print("Alinan Payload Lon: "); Serial.println(payload_lon, 6);
    Serial.print("Gyro X: "); Serial.println(gyro_x, 4);
    Serial.print("Gyro Y: "); Serial.println(gyro_y, 4);
    Serial.print("Gyro Z: "); Serial.println(gyro_z, 4);
    Serial.print("Acc X: "); Serial.println(accel_x, 4);
    Serial.print("Acc Y: "); Serial.println(accel_y, 4);
    Serial.print("Acc Z: "); Serial.println(accel_z, 4);
    Serial.print("Angle: "); Serial.print(relPitch, 2); Serial.println(" °");
    Serial.print("Status: 0x"); Serial.println(buf[74], HEX);
    Serial.print("Checksum: 0x"); Serial.println(buf[75], HEX);
    Serial.print("Footer: 0x"); Serial.print(buf[76], HEX);
    Serial.print(" 0x"); Serial.println(buf[77], HEX);

    delay(200);
}
