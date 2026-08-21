// #include <Arduino.h>
// #include <SPI.h>
// #include <LoRa.h>

// #define SCK 5
// #define MISO 19
// #define MOSI 27
// #define SS 18
// #define RST 23
// #define DIO 26
// #define BAND 915E6

// int contador = 0;
// void setup() {
//   Serial.begin(115200);
//   SPI.begin(SCK,MISO,MOSI,SS);
//   LoRa.setPins(SS,RST,DIO);
//   if (!LoRa.begin(915E6))
//   {
//     Serial.print("No inicio el radio");
//     while (1);
//   }
//   Serial.print("Radio inicializado exitosamente");
//   LoRa.setFrequency(915E6);
// }

// void loop() {
//   int a;
//   while(LoRa.beginPacket() == 0)
//   {
//     Serial.print("esperando por el radio...");
//     delay(100);
//   }
//   Serial.print("enviando data");
//   Serial.println(contador);

//   LoRa.beginPacket();
//   Serial.println("incio paquete");
//   LoRa.print("hola");
//   Serial.println("escribio hola");
//   LoRa.print(contador);
//   Serial.println("incremetno contador");
//   a = LoRa.endPacket();
//   if (a) Serial.println("transmision exitosa");
//   else Serial.println("error de tx");
//   Serial.println("termino paquete");
//   contador++;

//   delay(1000);
// }

// Only supports SX1276/SX1278
//
// Maquina de estados de 3 pasos que se repite cada TX_INTERVAL_MS:
//
//     PRUNING  ->  BUNDLING  ->  CHIRP  -+
//        ^                               |
//        +-------------------------------+
//
//   PRUNING   limpia las muestras del sensor (descarta outliers)
//   BUNDLING  arma la cadena CSV que se va a enviar
//   CHIRP     saca el paquete al aire (el chirp del SX1276)
//
// El GPS y el sensor se leen de fondo en cada vuelta del loop, en cualquier
// estado, porque no se puede dejar de atender el UART del GPS.

#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <ClosedCube_HDC1080.h>
#include "LoRaBoards.h"

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           915.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   17
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif

// Nombre con el que se identifica este transmisor en el aire
#define NODE_ID                     "clark"

// Cada cuanto se ejecuta un ciclo completo de la maquina de estados (ms)
#define TX_INTERVAL_MS              5000

// Poner en 1 para ver la trama NMEA cruda por el monitor serie.
// Ojo: ensucia bastante la salida legible.
#define DEBUG_NMEA                  0

// HDC1080: direccion I2C fija de fabrica
#define HDC1080_ADDR                0x40

// Muestreo del sensor: 20 muestras cada 250 ms = ventana de 5 s,
// justo lo que dura un ciclo. Sin delay(), para no perder datos del GPS.
#define SENSOR_SAMPLES              20
#define SENSOR_SAMPLE_MS            250

// Pruning: se descartan las muestras que se alejen mas de esto (en
// desviaciones estandar) del promedio del grupo.
#define PRUNE_SIGMA                 2.0f


#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples"
#endif

#ifndef HAS_GPS
#error "Esta placa no tiene GPS habilitado en utilities.h (falta HAS_GPS)"
#endif


// ---------------------------------------------------------------------------
// Estados
// ---------------------------------------------------------------------------
enum State {
    STATE_PRUNING,
    STATE_BUNDLING,
    STATE_CHIRP,
};

static State    state = STATE_PRUNING;
static uint32_t lastCycle = 0;

TinyGPSPlus         gps;
ClosedCube_HDC1080  hdc1080;

// ---- muestras crudas del sensor (buffer circular) ----
static bool     sensorOk = false;
static float    tempBuf[SENSOR_SAMPLES];
static float    humBuf[SENSOR_SAMPLES];
static uint8_t  sampleIdx = 0;
static uint8_t  sampleCount = 0;      // se satura en SENSOR_SAMPLES
static uint32_t lastSample = 0;

// ---- resultado del ciclo actual ----
static int      counter = 0;
static char     payload[128];         // lo arma BUNDLING, lo envia CHIRP
static bool     haveFix = false;
static bool     haveEnv = false;      // hubo muestras validas del sensor
static float    envTemp = 0;
static float    envHum = 0;
static uint8_t  keptTemp = 0;         // cuantas sobrevivieron al pruning
static uint8_t  keptHum = 0;


// ---------------------------------------------------------------------------
// Tareas de fondo (corren en cualquier estado)
// ---------------------------------------------------------------------------

// Lee todo lo que haya llegado del GPS y se lo pasa al parser NMEA.
// Hay que llamarla seguido: a 9600 baud el buffer del UART se desborda
// si dejamos de leer durante los segundos que hay entre transmisiones.
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

// Guarda una muestra cada SENSOR_SAMPLE_MS. No bloquea.
static void sampleSensor()
{
    if (!sensorOk || millis() - lastSample < SENSOR_SAMPLE_MS) {
        return;
    }
    lastSample = millis();

    float t = hdc1080.readTemperature();
    float h = hdc1080.readHumidity();

    // Rango del HDC1080: -40..125 C, 0..100 %RH. Un 125 C suele significar
    // que el sensor dejo de responder y el bus devolvio 0xFFFF.
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


// ---------------------------------------------------------------------------
// Estado 1: PRUNING
// ---------------------------------------------------------------------------

// Promedia el buffer descartando los valores que se alejan mas de
// PRUNE_SIGMA desviaciones estandar. Devuelve en *kept cuantos quedaron.
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

    // Segunda pasada: promedia solo las muestras que caen dentro del margen
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


// ---------------------------------------------------------------------------
// Estado 2: BUNDLING
// ---------------------------------------------------------------------------

// Arma el texto que viaja por el aire. Va formateado y con saltos de linea
// a proposito: cualquier receptor que simplemente imprima lo que recibe
// (aunque no sea nuestro codigo) lo muestra legible, sin tener que parsear.
//
//     clark #31
//     GPS: 6.240837, -75.590648
//     Alt: 1546.1m | Sats: 6
//     Time: 20:28:46 UTC
//     Temp: 24.7C | Hum: 58.3%
//
// El costo es el tamano: ~100 bytes contra los ~56 del CSV, o sea mas
// tiempo al aire por paquete. Ver la nota de airtime en printReadable().
static void doBundling()
{
    char env[40];
    if (haveEnv) {
        snprintf(env, sizeof(env), "Temp: %.1fC | Hum: %.1f%%", envTemp, envHum);
    } else {
        snprintf(env, sizeof(env), "Temp: n/d | Hum: n/d");
    }

    unsigned sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    if (haveFix) {
        snprintf(payload, sizeof(payload),
                 NODE_ID " #%d\n"
                 "GPS: %.6f, %.6f\n"
                 "Alt: %.1fm | Sats: %u\n"
                 "Time: %02u:%02u:%02u UTC\n"
                 "%s",
                 counter,
                 gps.location.lat(),
                 gps.location.lng(),
                 gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
                 sats,
                 gps.time.isValid() ? gps.time.hour() : 0,
                 gps.time.isValid() ? gps.time.minute() : 0,
                 gps.time.isValid() ? gps.time.second() : 0,
                 env);
    } else {
        snprintf(payload, sizeof(payload),
                 NODE_ID " #%d\n"
                 "GPS: sin fix (%u sats a la vista)\n"
                 "%s",
                 counter, sats, env);
    }
}


// ---------------------------------------------------------------------------
// Estado 3: CHIRP
// ---------------------------------------------------------------------------

static void printReadable()
{
    Serial.println();
    Serial.println("---------- enviando ----------");
    Serial.println(payload);              // identico a lo que recibe el otro lado
    Serial.println("------------------------------");

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
    // A SF10/BW125/CR4-7 cada byte cuesta ~14 ms de aire. Si esto se acerca
    // al intervalo de envio, hay que subir TX_INTERVAL_MS o acortar el texto.
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

static void doChirp()
{
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();

    printReadable();
    drawScreen();
    counter++;
}


// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup()
{
    // setupBoards() ya arranca SerialGPS (Serial1) en GPS_RX_PIN/GPS_TX_PIN
    // a GPS_BAUD_RATE, y tambien hace Wire.begin(I2C_SDA=21, I2C_SCL=22)
    // mas un scan del bus (ahi deberia aparecer el 0x40 del HDC1080).
    setupBoards();
    // When the power is turned on, a delay is required.
    delay(1500);

    // Un ping al bus antes de usar el sensor: si no esta conectado, el bus
    // devuelve 0xFFFF y la libreria lo traduce a 125 C / 100 %RH, que parecen
    // lecturas validas. Mejor saberlo y mandar NA.
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

    // CRC activado: sin esto el receptor entrega paquetes con bits dañados
    // como si fueran validos (se veian cosas como "NOFYX" o "s?ts=0").
    LoRa.enableCrc();

    LoRa.disableInvertIQ();

    LoRa.setCodingRate4(7);
}

void loop()
{
    // De fondo, siempre: no se puede dejar de leer el GPS ni el sensor.
    feedGPS();
    sampleSensor();

    switch (state) {

    case STATE_PRUNING:
        // Aca tambien se espera a que toque el siguiente ciclo.
        if (millis() - lastCycle < TX_INTERVAL_MS) {
            break;
        }
        lastCycle = millis();
        doPruning();
        state = STATE_BUNDLING;
        break;

    case STATE_BUNDLING:
        doBundling();
        state = STATE_CHIRP;
        break;

    case STATE_CHIRP:
        doChirp();
        state = STATE_PRUNING;
        break;
    }
}
