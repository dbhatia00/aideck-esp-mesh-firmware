#include "cpx_receive.h"
#include "spi_transport.h"
#include "mesh_network.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "CPX_Receive";

xQueueHandle cpxRxQueue;

static void cpxPacketReceivedTask(void *arg) {
    CPXRoutablePacket_t packet;

    while (1) {
        if (xQueueReceive(cpxRxQueue, &packet, portMAX_DELAY) == pdTRUE) {
            // Process the received packet
            if (packet.route.source == CPX_T_STM32 && packet.route.function == CPX_F_APP) {
                if (packet.dataLength == sizeof(TelemetryData_t)) {
                    TelemetryData_t telemetryData;
                    memcpy(&telemetryData, packet.data, sizeof(TelemetryData_t));

                    // Log the received telemetry data
                    ESP_LOGI(TAG, "Received telemetry from CF: Voltage=%.2fV, Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°",
                             telemetryData.batteryVoltage,
                             telemetryData.roll,
                             telemetryData.pitch,
                             telemetryData.yaw);

                    // Forward the telemetry data over the mesh network
                    mesh_send_data((uint8_t *)&telemetryData, sizeof(TelemetryData_t));
                } else {
                    ESP_LOGW(TAG, "Unexpected data length from CF: %d", packet.dataLength);
                }
            }
        }
    }
}

void cpxReceiveInit(void) {
    // Create a queue to receive CPX packets from SPI transport
    cpxRxQueue = xQueueCreate(10, sizeof(CPXRoutablePacket_t));

    // Start the CPX packet received task
    xTaskCreate(cpxPacketReceivedTask, "cpxPacketReceivedTask", 4096, NULL, 5, NULL);
}

void sendTelemetryToCF(const TelemetryData_t *telemetryData) {
    CPXRoutablePacket_t packet;
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_APP, &packet.route);
    packet.route.lastPacket = true;
    packet.dataLength = sizeof(TelemetryData_t);
    memcpy(packet.data, telemetryData, packet.dataLength);

    // Send the packet over SPI transport
    spi_transport_send(&packet);
}
