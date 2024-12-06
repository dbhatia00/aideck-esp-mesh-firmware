// ESP32 Code
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cpx.h"
#include "wifi.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_event.h"
#include "esp_mesh_internal.h"
#include "com.h"  // Include com.h to use com_receive_app_blocking()
#include "mesh_network.h"  // Include the header for function prototypes

#define DEBUG_MODULE "ESP32_MESH"

static const char *TAG = "ESP32_MESH";

bool validate_wifi_state() {
    // Check if Wi-Fi is in station mode
    wifi_mode_t current_wifi_mode;
    if (esp_wifi_get_mode(&current_wifi_mode) != ESP_OK || current_wifi_mode != WIFI_MODE_STA) {
        ESP_LOGE(TAG, "Wi-Fi is not in station mode (current mode: %d)", current_wifi_mode);
        return false;
    }

    // Wi-Fi is ready
    ESP_LOGI(TAG, "Wi-Fi is in station mode and ready for ESP-MESH");
    return true;
}

bool wait_for_wifi_ready() {
    for (int i = 0; i < 100; i++) { // Retry 5 times
        if (validate_wifi_state()) {
            return true;
        }
        ESP_LOGW(TAG, "Wi-Fi not ready, retrying...");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
    ESP_LOGE(TAG, "Wi-Fi not ready after retries");
    return false;
}

void mesh_init() {
    ESP_LOGI(TAG, "Initializing ESP32 Mesh Communication");

    // Ensure Wi-Fi is in station mode
    wifi_mode_t current_wifi_mode;
    if (esp_wifi_get_mode(&current_wifi_mode) != ESP_OK || current_wifi_mode != WIFI_MODE_STA) {
        ESP_LOGE(TAG, "Wi-Fi is not in station mode. Setting station mode.");
        esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set Wi-Fi mode: %s", esp_err_to_name(err));
            return;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for mode transition
    }

    // Add a delay to ensure Wi-Fi subsystem is ready
    ESP_LOGI(TAG, "Delaying to ensure Wi-Fi readiness...");

    // Call this before mesh configuration
    if (!wait_for_wifi_ready()) {
        ESP_LOGE(TAG, "Wi-Fi not ready, aborting mesh initialization");
        return;
    }

    // Proceed with mesh initialization
    mesh_cfg_t mesh_cfg = MESH_INIT_CONFIG_DEFAULT();
    uint8_t mesh_id[6] = {0x7D, 0x0A, 0x2C, 0x9E, 0x33, 0x56};
    memcpy(mesh_cfg.mesh_id.addr, mesh_id, sizeof(mesh_id));

    mesh_cfg.channel = 0;  // Auto-select channel
    mesh_cfg.router.ssid_len = strlen("MESH_PLACEHOLDER");
    memcpy(mesh_cfg.router.ssid, "MESH_PLACEHOLDER", mesh_cfg.router.ssid_len);
    memset(mesh_cfg.router.password, 0, sizeof(mesh_cfg.router.password));
    mesh_cfg.crypto_funcs = NULL;

    mesh_cfg.mesh_ap.max_connection = 6; // Max devices in the mesh network
    memset(mesh_cfg.mesh_ap.password, 0, sizeof(mesh_cfg.mesh_ap.password)); // Open network
    
    ESP_LOGI(TAG, "Free heap before config: %d", esp_get_free_heap_size());

    // Log mesh configuration for debugging
    ESP_LOGI(TAG, "Mesh ID: %02X:%02X:%02X:%02X:%02X:%02X",
             mesh_cfg.mesh_id.addr[0], mesh_cfg.mesh_id.addr[1],
             mesh_cfg.mesh_id.addr[2], mesh_cfg.mesh_id.addr[3],
             mesh_cfg.mesh_id.addr[4], mesh_cfg.mesh_id.addr[5]);
    ESP_LOGI(TAG, "Channel: %d", mesh_cfg.channel);
    ESP_LOGI(TAG, "Router SSID Length: %d", mesh_cfg.router.ssid_len);

    // Initialize ESP-MESH
    esp_err_t err = esp_mesh_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mesh initialization failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Mesh Config: SSID=%s, SSID_LEN=%d, CHANNEL=%d",
         (char *)mesh_cfg.router.ssid, mesh_cfg.router.ssid_len, mesh_cfg.channel);
    ESP_LOGI(TAG, "Ensuring Wi-Fi stability...");
    vTaskDelay(2000 / portTICK_PERIOD_MS); // 2-second delay

    // Set mesh configuration
    err = esp_mesh_set_config(&mesh_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mesh_set_config failed: %s", esp_err_to_name(err));
        return;
    }

    // Start ESP-MESH
    err = esp_mesh_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mesh_start failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "ESP-MESH initialized and started successfully");

    // Create task to handle forwarding telemetry packets from COM to mesh
    if (xTaskCreate(com_to_mesh_task, "com_to_mesh_task", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create com_to_mesh_task");
    } else {
        ESP_LOGI(TAG, "com_to_mesh_task created successfully");
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
            ESP_LOGI(TAG, "Forwarding telemetry via ESP-MESH: DroneID=%d",
                     receivedData.droneID);

            ESP_LOGI(TAG, "Voltage=%.2fV, Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
                     receivedData.batteryVoltage, receivedData.roll,
                     receivedData.pitch, receivedData.yaw);

            // Forward the telemetry data to the mesh network
            //mesh_send_telemetry(&receivedData);
            vTaskDelay(10 / portTICK_PERIOD_MS);

        } else {
            ESP_LOGE(TAG, "Received data of unexpected length: %d, expected: %d", packet.dataLength, sizeof(TelemetryData_t));
        }
    }
}