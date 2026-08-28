// config.h
#ifndef CONFIG_H
#define CONFIG_H

// Radio & Hardware Configuration
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           915.0
#endif

#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   17
#endif

#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif

#ifndef HDC1080_ADDR
#define HDC1080_ADDR                0x40
#endif

// Node Identity & Intervals
#ifndef NODE_ID
#define NODE_ID                     "516163"
#endif

#ifndef TX_INTERVAL_MS
#define TX_INTERVAL_MS              10000
#endif

// Sensor & Sampling Settings
#ifndef SENSOR_SAMPLES
#define SENSOR_SAMPLES              8
#endif

#ifndef SENSOR_SAMPLE_MS
#define SENSOR_SAMPLE_MS            1000
#endif

#ifndef PRUNE_SIGMA
#define PRUNE_SIGMA                 2.0f
#endif

// Frame Format
#ifndef PAYLOAD_MAX
#define PAYLOAD_MAX                 320     // trama final; 229 bytes con bonificacion
#endif

#ifndef BASE_MAX
#define BASE_MAX                    96      // solo campos mandatorios; 72 tipico, 77 el peor caso
#endif

#ifndef CAESAR_SHIFT
#define CAESAR_SHIFT                5
#endif

// Debugging
#ifndef DEBUG_NMEA
#define DEBUG_NMEA                  0
#endif

#endif // CONFIG_H
