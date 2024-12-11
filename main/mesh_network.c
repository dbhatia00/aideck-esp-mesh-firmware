// ESP32 Code
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cpx.h"
#include "wifi.h"
#include "esp_log.h"
#include "esp_now.h"
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
    ESP_LOGI(TAG, "Wi-Fi is in station mode and ready for protocol init");
    return true;
}

bool wait_for_wifi_ready() {
    for (int i = 0; i < 10; i++) { // Retry 10 times
        if (validate_wifi_state()) {
            return true;
        }
        ESP_LOGW(TAG, "Wi-Fi not ready, retrying...");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
    ESP_LOGE(TAG, "Wi-Fi not ready after retries");
    return false;
}

void espnow_receive_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    ESP_LOGI(TAG, "Received data from MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    if (len == sizeof(TelemetryData_t)) {
        TelemetryData_t receivedTelemetry;
        memcpy(&receivedTelemetry, data, sizeof(TelemetryData_t));
        // Log the telemetry data
        ESP_LOGI(TAG, "Telemetry Received: DroneID=%d",
                     receivedTelemetry.droneID);

        ESP_LOGI(TAG, "Voltage=%.2fV, Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
                     receivedTelemetry.batteryVoltage, receivedTelemetry.roll,
                     receivedTelemetry.pitch, receivedTelemetry.yaw);
    } else {
        ESP_LOGW(TAG, "Received data of unexpected length: %d", len);
    }
}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send status to MAC %02X:%02X:%02X:%02X:%02X:%02X: %s",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
             status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}


void add_espnow_peer(const uint8_t *peer_mac) {
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = 0;  // Match Wi-Fi channel
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;  // No encryption for simplicity

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ESP-NOW peer");
    } else {
        ESP_LOGI(TAG, "ESP-NOW peer added successfully: %02X:%02X:%02X:%02X:%02X:%02X",
                 peer_mac[0], peer_mac[1], peer_mac[2], peer_mac[3], peer_mac[4], peer_mac[5]);
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
            ESP_LOGD(TAG, "Forwarding telemetry via ESP-MESH: DroneID=%d",
                     receivedData.droneID);

            ESP_LOGD(TAG, "Voltage=%.2fV, Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
                     receivedData.batteryVoltage, receivedData.roll,
                     receivedData.pitch, receivedData.yaw);

            // Forward the telemetry data to the mesh network

            // Send telemetry over ESP-NOW
            uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            esp_err_t err = esp_now_send(broadcast_mac, (uint8_t *)&receivedData, sizeof(TelemetryData_t));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_now_send failed with error: %s", esp_err_to_name(err));
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);

        } else {
            ESP_LOGE(TAG, "Received data of unexpected length: %d, expected: %d", packet.dataLength, sizeof(TelemetryData_t));
        }
    }
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
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Call this before mesh configuration
    if (!wait_for_wifi_ready()) {
        ESP_LOGE(TAG, "Wi-Fi not ready, aborting mesh initialization");
        return;
    }

    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW initialization failed");
        return;
    }

    ESP_LOGI(TAG, "Successfully Initialized ESP-NOW! Setting up callbacks and peers...");
    esp_now_register_recv_cb(espnow_receive_cb);
    esp_now_register_send_cb(espnow_send_cb);

    uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    add_espnow_peer(broadcast_mac);

    // Log free heap for debugging purposes
    ESP_LOGI(TAG, "Free heap before task creation: %d", esp_get_free_heap_size());

    // Create task to handle forwarding telemetry packets from COM to mesh
    if (xTaskCreate(com_to_mesh_task, "com_to_mesh_task", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create com_to_mesh_task");
    } else {
        ESP_LOGI(TAG, "com_to_mesh_task created successfully");
    }
}
