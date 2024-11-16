#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

typedef struct {
    float batteryVoltage;
    float roll;
    float pitch;
    float yaw;
} __attribute__((packed)) TelemetryData_t;

#endif // TELEMETRY_H
