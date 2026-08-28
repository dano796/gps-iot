// state machine (pruning, bundling, transmit) that reads GPS and HDC1080 sensor data, prunes outliers, bundles into a JSON payload, and transmits via LoRa
// roy sandoval ID: 00516163 

#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <ClosedCube_HDC1080.h>
#include "LoRaBoards.h"
#include "config.h"
#include <MD5Builder.h>
#include <CJSON.h> 

#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples"
#endif

#ifndef HAS_GPS
#error "Esta placa no tiene GPS habilitado en utilities.h (falta HAS_GPS)"
#endif

// state machine 
enum State {
    STATE_PRUNING,
    STATE_BUNDLING,
    STATE_TRANSMIT,
};

static State    state = STATE_PRUNING;
static uint32_t lastCycle = 0;

TinyGPSPlus         gps;
ClosedCube_HDC1080  hdc1080;

static bool     sensorOk = false;
static float    tempBuf[SENSOR_SAMPLES];
static float    humBuf[SENSOR_SAMPLES];
static uint8_t  sampleIdx = 0;
static uint8_t  sampleCount = 0;      
static uint32_t lastSample = 0;

static int      counter = 0;
static char     payload[PAYLOAD_MAX];  
static bool     haveFix = false;
static bool     haveEnv = false;      
static float    envTemp = 0;
static float    envHum = 0;
static uint8_t  keptTemp = 0;        
static uint8_t  keptHum = 0;

//chirp
static void feedGPS()
{
    while (SerialGPS.available()) {
        char c = SerialGPS.read();
        gps.encode(c);
#if DEBUG_NMEA
        Serial.write(c);
#endif
    }
}

static void sampleSensor()
{
    if (!sensorOk || millis() - lastSample < SENSOR_SAMPLE_MS) {
        return;
    }
    lastSample = millis();

    float t = hdc1080.readTemperature();
    float h = hdc1080.readHumidity();

    if (isnan(t) || isnan(h) || t < -40.0f || t > 124.0f || h < 0.0f || h > 100.0f) {
        return;
    }

    tempBuf[sampleIdx] = t;
    humBuf[sampleIdx]  = h;
    sampleIdx = (sampleIdx + 1) % SENSOR_SAMPLES;
    if (sampleCount < SENSOR_SAMPLES) {
        sampleCount++;
    }
}

// state 1: pruning 

// averages the samples in buf[] and prunes outliers that are more than PRUNE_SIGMA standard deviations away from the mean
static float pruneAverage(const float *buf, uint8_t n, uint8_t *kept)
{
    float sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += buf[i];
    }
    float mean = sum / n;

    float var = 0;
    for (uint8_t i = 0; i < n; i++) {
        var += (buf[i] - mean) * (buf[i] - mean);
    }
    float sd = sqrtf(var / n);

    // averages samples within PRUNE_SIGMA standard deviations of the mean
    float sumOk = 0;
    uint8_t n_ok = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (fabsf(buf[i] - mean) <= PRUNE_SIGMA * sd) {
            sumOk += buf[i];
            n_ok++;
        }
    }
    *kept = n_ok;
    return n_ok ? (sumOk / n_ok) : mean;
}

static void doPruning()
{
    haveFix = gps.location.isValid();

    if (sampleCount == 0) {
        haveEnv = false;
        keptTemp = keptHum = 0;
        return;
    }
    envTemp = pruneAverage(tempBuf, sampleCount, &keptTemp);
    envHum  = pruneAverage(humBuf,  sampleCount, &keptHum);
    haveEnv = true;
}

// state 2: bundling

//bonus: caesar cipher with shift CAESAR_SHIFT (see config.h) 
static void caesarEncode(const char *in, char *out, size_t outSize)
{
    size_t i = 0;
    while (in[i] != '\0' && i < outSize - 1) {
        out[i] = ((in[i] - 32 + CAESAR_SHIFT) % 95) + 32;
        i++;
    }
    out[i] = '\0';
}

// hexadecimal md5(message + id) 
static void md5Hex(const char *msg, char *out, size_t outSize)
{
    MD5Builder md5;
    md5.begin();
    md5.add(msg);
    md5.add(NODE_ID);
    md5.calculate();
    snprintf(out, outSize, "%s", md5.toString().c_str());
}

//creates the json payload with the gps and sensor data, plus the caesar cipher and md5 hash
static void doBundling()
{
    char base[BASE_MAX];
    char cifrado[BASE_MAX];
    char integridad[33];            // 32 hex + '\0'

    snprintf(base, sizeof(base),
             "{\"id\": \"" NODE_ID "\",\"lat\": %.6f, \"lon\": %.6f, \"temperatura\": %.1f}",
             gps.location.lat(), gps.location.lng(), envTemp);

    caesarEncode(base, cifrado, sizeof(cifrado));
    md5Hex(base, integridad, sizeof(integridad));

    base[strlen(base) - 1] = '\0';
    snprintf(payload, sizeof(payload),
             "%s, \"metadata\":{\"campoCifrado\":\"%s\",\"campoIntegridad\":\"%s\"}}",
             base, cifrado, integridad);
}

//state 3: transmit 
static void printReadable()
{
    Serial.println();
    Serial.println("-- enviando --");
    Serial.println(payload);              // identico a lo que recibe el otro lado
    Serial.println("--");

    if (haveEnv) {
        Serial.printf("Pruning...... temp %u/%u muestras, hum %u/%u muestras\n",
                      keptTemp, sampleCount, keptHum, sampleCount);
    } else {
        Serial.printf("Pruning...... sin muestras (%s)\n",
                      sensorOk ? "midiendo" : "HDC1080 ausente");
    }
    if (!haveFix) {
        Serial.printf("NMEA......... %lu sentencias validas, %lu con checksum malo\n",
                      (unsigned long)gps.passedChecksum(),
                      (unsigned long)gps.failedChecksum());
    }
    Serial.printf("Payload...... %u bytes\n", (unsigned)strlen(payload));
}

static void drawScreen()
{
    if (!u8g2) {
        return;
    }
    char buf[32];
    u8g2->clearBuffer();

    if (haveFix) {
        snprintf(buf, sizeof(buf), "#%d Sat:%u", counter,
                 (unsigned)(gps.satellites.isValid() ? gps.satellites.value() : 0));
        u8g2->drawStr(0, 12, buf);
        snprintf(buf, sizeof(buf), "Lat:%.6f", gps.location.lat());
        u8g2->drawStr(0, 26, buf);
        snprintf(buf, sizeof(buf), "Lon:%.6f", gps.location.lng());
        u8g2->drawStr(0, 40, buf);
    } else {
        snprintf(buf, sizeof(buf), "#%d sin fix", counter);
        u8g2->drawStr(0, 12, buf);
        snprintf(buf, sizeof(buf), "Sats:%u",
                 (unsigned)(gps.satellites.isValid() ? gps.satellites.value() : 0));
        u8g2->drawStr(0, 26, buf);
        u8g2->drawStr(0, 40, "Buscando...");
    }

    if (haveEnv) {
        snprintf(buf, sizeof(buf), "T:%.1fC H:%.0f%%", envTemp, envHum);
    } else {
        snprintf(buf, sizeof(buf), "%s", sensorOk ? "Sensor: midiendo" : "Sensor: ausente");
    }
    u8g2->drawStr(0, 54, buf);
    u8g2->sendBuffer();
}

static void doTransmit()
{
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();

    printReadable();
    drawScreen();
    counter++;
}

void setup()
{
    setupBoards();
    // When the power is turned on, a delay is required.
    delay(1500);

    // ping before using sensor 
    Wire.beginTransmission(HDC1080_ADDR);
    sensorOk = (Wire.endTransmission() == 0);
    if (sensorOk) {
        hdc1080.begin(HDC1080_ADDR);
        Serial.println("HDC1080 detectado en 0x40");
    } else {
        Serial.println("WARN: HDC1080 no responde en 0x40 (se enviara NA)");
    }

#ifdef  RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif

    Serial.println("LoRa Sender + GPS + HDC1080");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(CONFIG_RADIO_FREQ * 1000000)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    LoRa.setTxPower(CONFIG_RADIO_OUTPUT_POWER);

    LoRa.setSignalBandwidth(CONFIG_RADIO_BW * 1000);

    LoRa.setSpreadingFactor(10);

    LoRa.setPreambleLength(16);

    LoRa.setSyncWord(0xAB);
    LoRa.enableCrc();

    LoRa.disableInvertIQ();

    LoRa.setCodingRate4(7);
}

void loop()
{
    feedGPS();
    sampleSensor();

    switch (state) {

    case STATE_PRUNING:
        if (millis() - lastCycle < TX_INTERVAL_MS) {
            break;
        }
        lastCycle = millis();
        doPruning();
        state = STATE_BUNDLING;
        break;

    case STATE_BUNDLING:
        doBundling();
        state = STATE_TRANSMIT;
        break;

    case STATE_TRANSMIT:
        doTransmit();
        state = STATE_PRUNING;
        break;
    }
}
