#ifndef MESH_NETWORK_HANDLER_H
#define MESH_NETWORK_HANDLER_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cpx.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "com.h"  // Include com.h to use com_receive_app_blocking()

#define DEBUG_MODULE "ESP32_MESH"
typedef struct {
    uint8_t droneID;
    double batteryVoltage;
    double roll;
    double pitch;
    double yaw;
} __attribute__((packed)) TelemetryData_t;


// Function prototypes
void mesh_init();
void com_to_mesh_task(void *arg);

#endif // MESH_NETWORK_HANDLER_H
