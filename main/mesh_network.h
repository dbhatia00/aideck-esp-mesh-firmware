#ifndef MESH_NETWORK_HANDLER_H
#define MESH_NETWORK_HANDLER_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cpx.h"
#include "esp_log.h"
#include "esp_now.h"
#include "com.h"

#define DEBUG_MODULE "ESP32_MESH"
typedef struct {
    uint8_t droneID;
    double batteryVoltage;
    double roll;
    double pitch;
    double yaw;
} __attribute__((packed)) TelemetryData_t;


void mesh_init();

#endif // MESH_NETWORK_HANDLER_H
