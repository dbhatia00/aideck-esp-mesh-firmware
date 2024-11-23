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
#include "mesh_network.h"  // Include the header for function prototypes

#define DEBUG_MODULE "ESP32_MESH"

static const char *TAG = "ESP32_MESH";

// Function to initialize mesh network
void mesh_init() {
    // Mesh network initialization
    ESP_LOGI(TAG, "Initializing ESP32 Mesh Communication");
    
    // Create task to handle forwarding telemetry packets from COM to mesh
    if (xTaskCreate(com_to_mesh_task, "com_to_mesh_task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create com_to_mesh_task");
    } else {
        ESP_LOGW(TAG, "com_to_mesh_task created");
    }
}

// Function to send telemetry data over mesh
void mesh_send_telemetry(const TelemetryData_t* telemetryData) {
    mesh_data_t mesh_data;
    mesh_data.data = (uint8_t *)telemetryData;
    mesh_data.size = sizeof(TelemetryData_t);
    mesh_data.proto = MESH_PROTO_BIN;
    mesh_data.tos = MESH_TOS_P2P;

    mesh_addr_t dest_addr;
    memset(dest_addr.addr, 0, 6); // Set to broadcast address to send to all nodes

    esp_err_t err = esp_mesh_send(&dest_addr, &mesh_data, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error sending mesh data: %s", esp_err_to_name(err));
    } else {
        ESP_LOGW(TAG, "Data sent over mesh, length: %d", (int)mesh_data.size);
    }
}

// Task to handle forwarding packets from COM to mesh
void com_to_mesh_task(void *arg) {
    esp_routable_packet_t packet;

    while (1) {
        // Blocking call to wait for a packet from the application queue
        com_receive_app_blocking(&packet);

        // Assuming the packet contains telemetry data
        if (packet.dataLength == sizeof(TelemetryData_t)) {
            TelemetryData_t receivedData;
            memcpy(&receivedData, packet.data, sizeof(TelemetryData_t));

            // Log the telemetry data
            ESP_LOGI(TAG, "Forwarding telemetry to mesh: DroneID=%d\n",
                     receivedData.droneID);

            ESP_LOGI(TAG, "Voltage=%.2fV, Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
                     receivedData.batteryVoltage, receivedData.roll,
                     receivedData.pitch, receivedData.yaw);

            // Forward the telemetry data to the mesh network
        } else {
            ESP_LOGE(TAG, "Received data of unexpected length: %d, expected: %d", packet.dataLength, sizeof(TelemetryData_t));
        }
    }
}
